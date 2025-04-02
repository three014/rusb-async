use atomic_waker::AtomicWaker;
use pin_project_lite::pin_project;
use ptr::Ptr;
use rusb::constants::{
    LIBUSB_ERROR_ACCESS, LIBUSB_ERROR_BUSY, LIBUSB_ERROR_INTERRUPTED, LIBUSB_ERROR_INVALID_PARAM,
    LIBUSB_ERROR_IO, LIBUSB_ERROR_NO_DEVICE, LIBUSB_ERROR_NO_MEM, LIBUSB_ERROR_NOT_FOUND,
    LIBUSB_ERROR_NOT_SUPPORTED, LIBUSB_ERROR_OVERFLOW, LIBUSB_ERROR_PIPE, LIBUSB_ERROR_TIMEOUT,
    LIBUSB_TRANSFER_ADD_ZERO_PACKET, LIBUSB_TRANSFER_CANCELLED, LIBUSB_TRANSFER_COMPLETED,
    LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_NO_DEVICE, LIBUSB_TRANSFER_OVERFLOW,
    LIBUSB_TRANSFER_SHORT_NOT_OK, LIBUSB_TRANSFER_STALL, LIBUSB_TRANSFER_TIMED_OUT,
    LIBUSB_TRANSFER_TYPE_BULK, LIBUSB_TRANSFER_TYPE_CONTROL, LIBUSB_TRANSFER_TYPE_INTERRUPT,
    LIBUSB_TRANSFER_TYPE_ISOCHRONOUS,
};
use rusb::ffi::{libusb_iso_packet_descriptor, libusb_transfer};
use state::{State, StateRepr};
use std::future::Future;
use std::mem::{ManuallyDrop, MaybeUninit};
use std::pin::Pin;
use std::ptr::NonNull;
use std::sync::Arc;
use std::task::{Poll, ready};
use std::time::Duration;
use usb::{
    libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer, libusb_submit_transfer,
};

#[cfg(feature = "zerocopy")]
use zerocopy_derive::*;

#[cfg(all(not(test), feature = "dma"))]
pub use dma::{AllocError, DeviceHandleExt};

pub use usb::UsbMemMut;

#[cfg(all(not(test), feature = "dma"))]
mod dma;
mod ptr;
mod state;
mod usb;
mod cancel {
    use std::pin::pin;
    use flag::Flag;

    #[derive(Clone)]
    pub struct CancellationToken(Flag);

    impl CancellationToken {
        pub fn new() -> Self {
            Self(Flag::new())
        }

        pub fn cancel(&self) {
            self.0.signal();
        }

        pub fn is_cancelled(&self) -> bool {
            self.0.is_set()
        }
    }

    impl Future for CancellationToken {
        type Output = ();

        fn poll(
            mut self: std::pin::Pin<&mut Self>,
            cx: &mut std::task::Context<'_>,
        ) -> std::task::Poll<Self::Output> {
            pin!(&mut self.0).poll(cx)
        }
    }
}

pub use cancel::CancellationToken;

#[cfg(test)]
mod sandbox;

#[cfg_attr(
    feature = "zerocopy",
    derive(IntoBytes, FromBytes, Immutable, KnownLayout)
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct ControlPacket {
    pub bm_request_type: u8,
    pub b_request: u8,
    pub w_value: u16,
    pub w_index: u16,
    pub w_length: u16,
}

#[cfg_attr(
    feature = "zerocopy",
    derive(IntoBytes, FromZeros, Immutable, KnownLayout)
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TransferStatus {
    Completed = 0,
    Error = LIBUSB_TRANSFER_ERROR as u8,
    TimedOut = LIBUSB_TRANSFER_TIMED_OUT as u8,
    Cancelled = LIBUSB_TRANSFER_CANCELLED as u8,
    Stall = LIBUSB_TRANSFER_STALL as u8,
    NoDevice = LIBUSB_TRANSFER_NO_DEVICE as u8,
    Overflow = LIBUSB_TRANSFER_OVERFLOW as u8,
}

impl TransferStatus {
    pub const fn from_i32(status: i32) -> Option<Self> {
        use TransferStatus::*;
        match status {
            LIBUSB_TRANSFER_COMPLETED => Some(Completed),
            LIBUSB_TRANSFER_ERROR => Some(Error),
            LIBUSB_TRANSFER_TIMED_OUT => Some(TimedOut),
            LIBUSB_TRANSFER_CANCELLED => Some(Cancelled),
            LIBUSB_TRANSFER_STALL => Some(Stall),
            LIBUSB_TRANSFER_NO_DEVICE => Some(NoDevice),
            LIBUSB_TRANSFER_OVERFLOW => Some(Overflow),
            _ => None,
        }
    }
}

bitflags::bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct TransferFlags: u8 {
        const NONE = 0;
        const SHORT_NOT_OK = LIBUSB_TRANSFER_SHORT_NOT_OK;
        const ADD_ZERO_PACKET = LIBUSB_TRANSFER_ADD_ZERO_PACKET;
    }
}

pub trait IsoPacket {
    fn len(&self) -> u32;
    fn actual_len(&self) -> u32 {
        0
    }
    fn status(&self) -> TransferStatus {
        TransferStatus::Completed
    }
}

impl IsoPacket for libusb_iso_packet_descriptor {
    fn len(&self) -> u32 {
        self.length
    }

    fn actual_len(&self) -> u32 {
        self.actual_length
    }

    fn status(&self) -> TransferStatus {
        TransferStatus::from_i32(self.status).unwrap()
    }
}

/// The data that the `user_data` field points to in a
/// libusb transfer. When allocating a transfer struct,
/// we specify more iso packets than necessary so we can
/// store this information at the end of the struct.
///
/// Since an allocated transfer is never moved during its
/// lifetime, we can specify the pointer to this struct
/// directly.
#[repr(C)]
struct Footer {
    waker: AtomicWaker,
    state: State,
    num_iso_packets: u16,
    buf: Option<UsbMemMut>,
}

impl Footer {
    #[inline]
    const fn new(num_iso_packets: u16) -> Self {
        Self {
            waker: AtomicWaker::new(),
            state: State::new_idle(),
            num_iso_packets,
            buf: None,
        }
    }

    #[inline]
    const fn ref_from_transfer(transfer: &libusb_transfer) -> &Self {
        unsafe { &*transfer.user_data.cast::<Self>() }
    }

    #[inline]
    const fn raw_mut_from_transfer(transfer: &mut libusb_transfer) -> *mut Self {
        transfer.user_data.cast::<Self>()
    }

    #[inline]
    const fn mut_from_transfer(transfer: &mut libusb_transfer) -> &mut Self {
        unsafe { &mut *transfer.user_data.cast::<Self>() }
    }

    #[inline]
    const fn num_iso_packets(&self) -> u16 {
        self.num_iso_packets
    }

    #[inline]
    const fn waker(&self) -> &AtomicWaker {
        &self.waker
    }

    #[inline]
    const fn state(&self) -> &State {
        &self.state
    }

    #[inline]
    const fn take_buf(&mut self) -> Option<UsbMemMut> {
        self.buf.take()
    }

    #[inline]
    fn insert_buf(&mut self, mut buf: UsbMemMut) -> (*mut u8, usize) {
        let len = buf.len();
        buf.clear();
        let ptr = buf.as_mut_ptr();
        self.buf = Some(buf);
        (ptr, len)
    }

    #[inline]
    unsafe fn set_buf_len(&mut self, transfer_len: usize) {
        if let Some(buf) = self.buf.as_mut() {
            unsafe { buf.set_len(transfer_len) };
        }
    }
}

/// An owned libusb transfer struct.
///
/// The main idea here is: There are certain fields that can be accessed
/// while a transfer is active, and some fields that cannot be accessed
/// while a transfer is active. Fields that cannot be accessed during
/// an active transfer can rely on the `state` field of the [`Footer`]
/// to tell when it's okay to access the inner data.
///
/// To make this work, all pointers to transfer itself must be
/// `*const` pointers to satisfy the aliasing rule, while individual
pub struct LibusbTransfer2 {
    inner: Ptr<libusb_transfer>,
}

unsafe impl Send for LibusbTransfer2 {}

// We want to calculate the minimum number of
// packets needed to give us room at the end of
// the allocation to fit an aligned `Footer` struct.

// Size of one iso packet descriptor and one footer struct for brevity.
const PKT_SIZE: usize = size_of::<libusb_iso_packet_descriptor>();
const FOOTER_SIZE: usize = size_of::<Footer>();

// The most amount of packets the caller can logically request.
const PKT_ALIGNED_SIZE_OF_FOOTER: usize = FOOTER_SIZE.next_multiple_of(PKT_SIZE);
const FOOTER_SIZE_IN_PACKETS: usize = PKT_ALIGNED_SIZE_OF_FOOTER / PKT_SIZE;
const MAX_NUM_PACKETS: usize = u16::MAX as usize - FOOTER_SIZE_IN_PACKETS;

/// # Safety
///
/// The caller must ensure that `footer` points to
/// valid memory, although the memory is allowed to
/// contain uninitialized data.
#[inline]
const unsafe fn set_footer(
    mut footer: NonNull<MaybeUninit<Footer>>,
    num_packets: u16,
) -> *mut Footer {
    // SAFETY: Caller upholds that `footer` points to valid memory.
    unsafe { footer.as_mut().write(Footer::new(num_packets)) }
}

impl LibusbTransfer2 {
    pub fn new_with_zero_packets() -> Self {
        // SAFETY: libusb allocates the data, and returns a null ptr
        // if the allocation somehow failed.
        let ptr = unsafe { libusb_alloc_transfer(FOOTER_SIZE_IN_PACKETS as i32) };
        let mut transfer = Ptr::new(ptr).expect("libusb failed to allocate");

        // SAFETY: `transfer` was just allocated through FFI
        // and is a valid pointer to convert to an exclusive
        // reference.
        let footer_ptr = unsafe {
            let transfer = transfer.as_ptr();
            let start_of_iso = &raw mut (*transfer).iso_packet_desc;
            start_of_iso
                .cast::<libusb_iso_packet_descriptor>()
                .byte_add(4) // 4 bytes to account for start position of packet slice
                .cast::<MaybeUninit<Footer>>()
        };

        // SAFETY: Since we know this pointer points to a
        // valid allocation, we can dereference the pointer to
        // write in our new Footer.
        let footer = unsafe { set_footer(NonNull::new_unchecked(footer_ptr), 0) };

        // Finally, we write the footer's address into
        // the `user_data` field of the transfer.
        //
        // SAFETY: While yes we are creating a self-referential
        // struct, the transfer was allocated on the heap and
        // will never be moved for its lifetime. Therefore, we
        // can expect the footer data to never change its address.
        unsafe { transfer.as_mut().user_data = footer.cast() };

        // Set the callback functions and ensure that we don't
        // expose the "invalid packets", aka, our footer data.
        unsafe {
            transfer.as_mut().callback = transfer_callback2;
            transfer.as_mut().num_iso_packets = 0;
        }

        Self { inner: transfer }
    }

    pub fn new(num_isos: usize) -> Self {
        #[inline(never)]
        #[cold]
        fn too_many_packets(pkts: usize) -> ! {
            panic!(
                "caller requested more than the maximum allowed iso packets ({pkts} > {MAX_NUM_PACKETS})"
            )
        }

        if num_isos > MAX_NUM_PACKETS {
            too_many_packets(num_isos);
        }
        let aligned_packets = num_isos + FOOTER_SIZE_IN_PACKETS;

        // SAFETY: libusb allocates the data, and returns a null ptr
        // if the allocation somehow failed.
        let ptr = unsafe { libusb_alloc_transfer(aligned_packets as i32) };
        let mut transfer = Ptr::new(ptr).expect("libusb failed to allocate");

        // Calculate pointer offset from beginning of iso packet
        // array for storing the footer.
        //
        // SAFETY: `iso_packet_desc` has length `aligned_packets` which
        // was computed from the sum of at least `num_isos`. Therefore,
        // the memory we are accessing is in bounds.
        let first_unused_pkt_ptr = unsafe {
            let transfer = transfer.as_ptr();
            let start_of_iso = &raw mut (*transfer).iso_packet_desc;
            start_of_iso
                .cast::<libusb_iso_packet_descriptor>()
                .add(num_isos)
        };

        #[inline]
        const fn is_offset_either_zero_or_four(offset: usize) -> bool {
            matches!(offset, 0 | 4)
        }

        // Align the ptr for use with our footer.
        //
        // SAFETY: Like earlier, we are dealing with
        // in-bounds memory, and the returned address
        // will have an offset of either 0 or 4 bytes
        // from `first_unused_pkt_ptr`.
        let footer_ptr = unsafe {
            let offset = first_unused_pkt_ptr
                .cast::<u8>()
                .align_offset(align_of::<Footer>());
            debug_assert!(is_offset_either_zero_or_four(offset));
            first_unused_pkt_ptr
                .cast::<u8>()
                .add(offset)
                .cast::<MaybeUninit<Footer>>()
        };

        // SAFETY: Since we know this pointer points to a
        // valid allocation, we can dereference the pointer to
        // write in our new Footer.
        let footer = unsafe { set_footer(NonNull::new_unchecked(footer_ptr), num_isos as u16) };

        // Finally, we write the footer's address into
        // the `user_data` field of the transfer.
        //
        // SAFETY: While yes we are creating a self-referential
        // struct, the transfer was allocated on the heap and
        // will never be moved for its lifetime. Therefore, we
        // can expect the footer data to never change its address.
        unsafe { transfer.as_mut().user_data = footer.cast() };

        // Set the callback functions and ensure that we don't
        // expose the "invalid packets", aka, our footer data.
        unsafe {
            transfer.as_mut().callback = transfer_callback2;
            transfer.as_mut().num_iso_packets = num_isos as _;
        }

        Self { inner: transfer }
    }

    #[inline]
    const fn footer(&self) -> &Footer {
        // TODO: Is this really safe? We shouldn't be giving out
        // a shared reference to this when libusb has mutable
        // access to the transfer.
        Footer::ref_from_transfer(unsafe { self.inner.as_ref() })
    }

    #[inline]
    const fn footer_mut(&mut self) -> &mut Footer {
        // TODO: Is this really safe? We shouldn't be giving out
        // an exclusive reference to this when libusb also has
        // mutable access to the transfer.
        Footer::mut_from_transfer(unsafe { self.inner.as_mut() })
    }

    #[inline]
    pub const fn max_iso_packets(&self) -> u16 {
        self.footer().num_iso_packets
    }

    #[inline]
    const fn num_iso_packets(&self) -> u16 {
        // SAFETY: This value will never be set by
        // libusb; Rust's borrow checker ensures this
        // is a safe reference to read.
        unsafe { self.inner.as_ref().num_iso_packets as u16 }
    }

    #[inline]
    const fn footer_as_raw(&mut self) -> *mut Footer {
        Footer::raw_mut_from_transfer(unsafe { self.inner.as_mut() })
    }

    // # Safety
    //
    // Do not call this function during an isochronous transfer.
    // There is nothing preventing a data race here and libusb will
    // not care what safety measures you have in place.
    #[inline]
    const unsafe fn iso_packets_mut(&mut self) -> &mut [libusb_iso_packet_descriptor] {
        unsafe {
            let len = self.num_iso_packets() as usize;
            let ptr = self.inner.as_mut().iso_packet_desc.as_mut_ptr();
            std::slice::from_raw_parts_mut(ptr, len)
        }
    }

    // # Safety
    //
    // Do not call this function during an isochronous transfer.
    // There is nothing preventing a data race here and libusb will
    // not care what safety measures you have in place.
    #[inline]
    const unsafe fn iso_packets(&self) -> &[libusb_iso_packet_descriptor] {
        unsafe {
            let len = self.num_iso_packets() as usize;
            let ptr = self.inner.as_ref().iso_packet_desc.as_ptr();
            std::slice::from_raw_parts(ptr, len)
        }
    }

    /// Submits the transfer to libusb.
    ///
    /// Since the inner call to `libusb_submit_transfer` is thread-safe,
    /// this function can be called from any thread. However, trying
    /// to submit the same transfer twice before the first submission
    /// finishes will return a (harmless) error.
    fn submit(&self) -> rusb::Result<()> {
        let result = unsafe { libusb_submit_transfer(self.inner.as_ptr()) };
        if result != 0 {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    fn cancel(&self) -> rusb::Result<()> {
        let result = unsafe { libusb_cancel_transfer(self.inner.as_ptr()) };
        if result != 0 {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    /// Resets the transfer state and clears out old data.
    /// Upon exiting this function, the transfer is ready to be
    /// used for a new unrelated submission.
    ///
    /// # Safety
    ///
    /// `Self` must not be part of an active submission or else
    /// a data race could occur when libusb runs the callback function.
    #[inline]
    unsafe fn reset(&mut self) {
        let footer = Footer::mut_from_transfer(unsafe { self.inner.as_mut() });
        let num_packets = footer.num_iso_packets();
        *footer = Footer::new(num_packets);
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// To use this function correctly, `buf.len()` must be the length
    /// of the data the caller expects to receive or send. The remaining
    /// capacity will be ignored.
    ///
    /// # Safety
    ///
    /// This transfer must not be part of an active submission. Also,
    /// `dev_handle` must outlive this transfer.
    pub unsafe fn into_int<C: rusb::UsbContext>(
        mut self,
        dev_handle: &usb::DeviceHandle<C>,
        endpoint: u8,
        buf: UsbMemMut,
    ) -> Transfer2 {
        // SAFETY: Caller upholds the "not in active submission" rule.
        unsafe { self.reset() };
        let (ptr, len) = self.footer_mut().insert_buf(buf);

        unsafe {
            let transfer = self.inner.as_mut();
            transfer.dev_handle = dev_handle.as_raw();
            transfer.endpoint = endpoint;
            transfer.transfer_type = LIBUSB_TRANSFER_TYPE_INTERRUPT;
            transfer.timeout = u32::MAX;
            transfer.buffer = ptr;
            transfer.length = len as _;
        }

        Transfer2::new(self)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// To use this function correctly, `buf.len()` must be the length
    /// of the data the caller expects to receive or send. The remaining
    /// capacity will be ignored.
    ///
    /// # Safety
    ///
    /// This transfer must not be part of an active submission. Also,
    /// `dev_handle` must outlive this transfer.
    pub unsafe fn into_bulk<C: rusb::UsbContext>(
        mut self,
        dev_handle: &usb::DeviceHandle<C>,
        endpoint: u8,
        flags: TransferFlags,
        buf: UsbMemMut,
    ) -> Transfer2 {
        // SAFETY: Caller upholds the "not in active submission" rule.
        unsafe { self.reset() };
        let (ptr, len) = self.footer_mut().insert_buf(buf);

        unsafe {
            let transfer = self.inner.as_mut();
            transfer.dev_handle = dev_handle.as_raw();
            transfer.endpoint = endpoint;
            transfer.transfer_type = LIBUSB_TRANSFER_TYPE_BULK;
            transfer.timeout = 1000;
            transfer.buffer = ptr;
            transfer.length = len as _;
            transfer.flags = flags.bits();
        }

        Transfer2::new(self)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// To use this function correctly, `buf.len()` must be the length
    /// of the data the caller expects to receive or send. The remaining
    /// capacity will be ignored.
    ///
    /// This function has a few more constraints.
    /// The caller must ensure that:
    ///
    /// - `buf.len() >= 8`
    /// - `ControlPacket::w_length + std::mem::size_of::<ControlPacket>() == buf.capacity()`
    ///
    /// ALSO, the caller must make sure that the first 8 bytes of `buf`
    /// contain the [`ControlPacket`].
    ///
    /// # Safety
    ///
    /// This transfer must not be part of an active submission. Also,
    /// `dev_handle` must outlive this transfer.
    pub unsafe fn into_ctrl<C: rusb::UsbContext>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        buf: UsbMemMut,
        timeout: Duration,
    ) -> Transfer2 {
        // SAFETY: Caller upholds the "not in active submission" rule.
        unsafe { self.reset() };
        let (ptr, len) = self.footer_mut().insert_buf(buf);
        let timeout = timeout.clamp(Duration::ZERO, Duration::from_millis(u32::MAX as u64));

        unsafe {
            let transfer = self.inner.as_mut();
            transfer.dev_handle = dev_handle.as_raw();
            transfer.endpoint = 0;
            transfer.transfer_type = LIBUSB_TRANSFER_TYPE_CONTROL;
            transfer.timeout = timeout.as_millis() as u32;
            transfer.buffer = ptr;
            transfer.length = len as _;
        };

        Transfer2::new(self)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// To use this function correctly, `buf.len()` must be the length
    /// of the data the caller expects to receive or send. The remaining
    /// capacity will be ignored.
    ///
    /// # Safety
    ///
    /// This transfer must not be part of an active submission. Also,
    /// `dev_handle` must outlive this transfer.
    pub unsafe fn into_iso<C: rusb::UsbContext, T: IsoPacket>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        buf: UsbMemMut,
        iso_packets: impl ExactSizeIterator<Item = T>,
    ) -> Transfer2 {
        let num_iso_packets = iso_packets.len();
        assert!(num_iso_packets <= self.max_iso_packets() as usize);

        unsafe { self.reset() };
        let (ptr, len) = self.footer_mut().insert_buf(buf);

        iso_packets
            .map(|pkt| pkt.len())
            // SAFETY: We're not in an active transfer yet.
            .zip(unsafe { self.iso_packets_mut() })
            .for_each(|(len, pkt)| pkt.length = len);

        unsafe {
            let transfer = self.inner.as_mut();
            transfer.dev_handle = dev_handle.as_raw();
            transfer.endpoint = endpoint;
            transfer.transfer_type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
            transfer.timeout = 1000;
            transfer.buffer = ptr;
            transfer.length = len as _;
            transfer.num_iso_packets = num_iso_packets as _;
        }

        Transfer2::new(self)
    }

    /// # Safety
    ///
    /// Calling this during an active transfer will probably
    /// result in a data race. Don't trust anything this
    /// function returns until the transfer has fully completed.
    #[inline]
    const unsafe fn len(&self) -> usize {
        unsafe { self.inner.as_ref().actual_length as usize }
    }

    /// # Safety
    ///
    /// Calling this during an active transfer will probably
    /// result in a data race. Don't trust anything this
    /// function returns until the transfer has fully completed.
    #[inline]
    const unsafe fn status(&self) -> TransferStatus {
        unsafe { TransferStatus::from_i32(self.inner.as_ref().status).unwrap() }
    }

    #[inline]
    const fn is_ctrl(&self) -> bool {
        // SAFETY: This field is written to once, then never changed afterwards.
        matches!(self.kind(), rusb::TransferType::Control)
    }

    #[inline]
    const fn kind(&self) -> rusb::TransferType {
        match unsafe { self.inner.as_ref().transfer_type } {
            LIBUSB_TRANSFER_TYPE_CONTROL => rusb::TransferType::Control,
            LIBUSB_TRANSFER_TYPE_INTERRUPT => rusb::TransferType::Interrupt,
            LIBUSB_TRANSFER_TYPE_BULK => rusb::TransferType::Bulk,
            LIBUSB_TRANSFER_TYPE_ISOCHRONOUS => rusb::TransferType::Isochronous,
            _ => unreachable!(),
        }
    }
}

impl Drop for LibusbTransfer2 {
    fn drop(&mut self) {
        // First, we need to run the footer's
        // drop function since deallocating
        // the transfer won't call it.

        // SAFETY: This is the last time we even
        // look at the footer data, we promise.
        unsafe { std::ptr::drop_in_place(self.footer_as_raw()) };

        // SAFETY: libusb allocated this buffer,
        // so they get to free it.
        unsafe {
            libusb_free_transfer(self.inner.as_ptr());
        }
    }
}

unsafe fn handle_ready(transfer: &mut LibusbTransfer2) -> TransferStatus {
    unsafe {
        let transfer_len = if transfer.is_ctrl() {
            transfer.len() + size_of::<ControlPacket>()
        } else {
            transfer.len()
        };
        let status = transfer.status();

        transfer.footer_mut().set_buf_len(transfer_len);

        status
    }
}

pin_project! {
    pub struct Wait<'a> {
        #[pin]
        cancel: &'a mut CancellationToken,
        transfer: &'a mut LibusbTransfer2,
        is_cancelled: bool,
    }
}

impl Future for Wait<'_> {
    type Output = rusb::Result<TransferStatus>;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let mut this = self.project();
        let transfer = this.transfer;

        #[inline(never)]
        #[cold]
        fn err_idle() -> ! {
            panic!("polled during idle transfer state")
        }

        match transfer.footer().state().get() {
            StateRepr::Idle => err_idle(),
            StateRepr::Running if !*this.is_cancelled => {
                transfer.footer().waker().register(cx.waker());
                ready!(this.cancel.as_mut().poll(cx));
                *this.is_cancelled = true;
                transfer.cancel()?;
            }
            StateRepr::Running => {
                transfer.footer().waker().register(cx.waker());
            }
            // SAFETY: Transfer has completed, we are back
            // to being the only place that can read/write
            // to this data.
            StateRepr::Ready => unsafe {
                return Poll::Ready(Ok(handle_ready(transfer)));
            },
        }
        Poll::Pending
    }
}

pin_project! {
    pub struct SubmitAndWait<'a> {
        #[pin]
        cancel: &'a mut CancellationToken,
        transfer: &'a mut LibusbTransfer2,
        is_cancelled: bool,
    }
}

impl Future for SubmitAndWait<'_> {
    type Output = rusb::Result<TransferStatus>;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let mut this = self.project();
        let transfer = this.transfer;

        match transfer.footer().state().get() {
            StateRepr::Idle => {
                transfer.footer().waker().register(cx.waker());
                transfer.submit()?;
                if !transfer.footer().state().set_running() {
                    return Poll::Ready(Ok(unsafe { handle_ready(transfer) }));
                }
            }
            StateRepr::Running if !*this.is_cancelled => {
                transfer.footer().waker().register(cx.waker());
                ready!(this.cancel.as_mut().poll(cx));
                *this.is_cancelled = true;
                transfer.cancel()?;
            }
            StateRepr::Running => {
                transfer.footer().waker().register(cx.waker());
            }
            // SAFETY: Transfer has completed, we are back
            // to being the only place that can read/write
            // to this data.
            StateRepr::Ready => unsafe {
                return Poll::Ready(Ok(handle_ready(transfer)));
            },
        }
        Poll::Pending
    }
}

pub struct Transfer2 {
    inner: Option<LibusbTransfer2>,
}

impl Transfer2 {
    #[inline]
    pub const fn new(inner: LibusbTransfer2) -> Self {
        Self { inner: Some(inner) }
    }

    #[inline]
    pub fn is_running(&self) -> bool {
        self.inner
            .as_ref()
            .is_some_and(|transfer| transfer.footer().state().get() == StateRepr::Running)
    }

    #[inline]
    pub fn submit_and_wait<'a>(
        &'a mut self,
        cancel_token: &'a mut CancellationToken,
    ) -> SubmitAndWait<'a> {
        SubmitAndWait {
            cancel: cancel_token,
            transfer: self.inner.as_mut().unwrap(),
            is_cancelled: false,
        }
    }

    #[inline]
    pub fn wait<'a>(&'a mut self, cancel_token: &'a mut CancellationToken) -> Wait<'a> {
        Wait {
            cancel: cancel_token,
            transfer: self.inner.as_mut().unwrap(),
            is_cancelled: false,
        }
    }

    #[inline]
    pub fn try_submit(&self) -> rusb::Result<()> {
        let transfer = self.inner.as_ref().unwrap();
        let state = transfer.footer().state();
        transfer.submit()?;
        state.set_running();
        Ok(())
    }

    #[inline]
    pub fn kind(&self) -> rusb::TransferType {
        self.inner.as_ref().unwrap().kind()
    }

    /// Returns `None` if the transfer type is not isochronous or if
    /// the transfer is not ready.
    #[inline]
    pub fn iso_packets(&self) -> Option<&[impl IsoPacket]> {
        (rusb::TransferType::Isochronous == self.kind() && !self.is_running())
            .then(|| unsafe { self.inner.as_ref().unwrap().iso_packets() })
    }

    #[inline]
    pub fn cancel(&self) -> rusb::Result<()> {
        self.inner.as_ref().unwrap().cancel()
    }

    #[inline]
    pub fn into_parts(mut self) -> Option<(LibusbTransfer2, UsbMemMut)> {
        (!self.is_running()).then(|| {
            let mut transfer = self.inner.take().unwrap();
            let buf = transfer.footer_mut().take_buf().unwrap();
            (transfer, buf)
        })
    }
}

impl Drop for Transfer2 {
    fn drop(&mut self) {
        if self.is_running() {
            let transfer = self.inner.take().unwrap();
            _ = transfer.cancel();
            match default_runtime() {
                Some(rt) => rt.spawn_detached(Box::pin(std::future::poll_fn(move |cx| {
                    let footer = transfer.footer();
                    footer.waker().register(cx.waker());
                    match footer.state().get() {
                        StateRepr::Running => Poll::Pending,
                        StateRepr::Idle | StateRepr::Ready => Poll::Ready(()),
                    }
                }))),
                None => {
                    // We leak the memory so that libusb nor
                    // the USB device will write into freed
                    // memory.
                    let _transfer = ManuallyDrop::new(transfer);
                }
            }
        }
    }
}

trait AsyncRuntime: Send + Sync + std::fmt::Debug + 'static {
    fn spawn_detached(&self, fut: Pin<Box<dyn Future<Output = ()> + Send>>);
}

#[cfg(feature = "runtime-compio")]
#[derive(Debug)]
struct CompioRuntime;

#[cfg(feature = "runtime-compio")]
impl AsyncRuntime for CompioRuntime {
    fn spawn_detached(&self, fut: Pin<Box<dyn Future<Output = ()> + Send>>) {
        compio_runtime::spawn(fut).detach();
    }
}

#[allow(unreachable_code)]
fn default_runtime() -> Option<Arc<dyn AsyncRuntime>> {
    #[cfg(feature = "runtime-tokio")]
    {
        return ::tokio::runtime::Handle::try_current()
            .ok()
            .map(|handle| Arc::new(handle) as Arc<dyn AsyncRuntime>);
    }

    #[cfg(feature = "runtime-compio")]
    {
        if ::compio_runtime::Runtime::try_with_current(|_| {}).is_ok() {
            Some(Arc::new(CompioRuntime) as Arc<dyn AsyncRuntime>)
        } else {
            None
        }
    }

    #[cfg(not(any(feature = "runtime-tokio", feature = "runtime-compio")))]
    None
}

#[cfg(feature = "runtime-tokio")]
impl AsyncRuntime for tokio::runtime::Handle {
    fn spawn_detached(&self, fut: Pin<Box<dyn Future<Output = ()> + Send>>) {
        self.spawn(fut);
    }
}

extern "system" fn transfer_callback2(transfer: *mut libusb_transfer) {
    let footer = Footer::ref_from_transfer(unsafe { &*transfer });
    footer.state().set_ready();
    footer.waker().wake();
}

pub(crate) fn from_libusb(err: i32) -> rusb::Error {
    match err {
        LIBUSB_ERROR_IO => rusb::Error::Io,
        LIBUSB_ERROR_INVALID_PARAM => rusb::Error::InvalidParam,
        LIBUSB_ERROR_ACCESS => rusb::Error::Access,
        LIBUSB_ERROR_NO_DEVICE => rusb::Error::NoDevice,
        LIBUSB_ERROR_NOT_FOUND => rusb::Error::NotFound,
        LIBUSB_ERROR_BUSY => rusb::Error::Busy,
        LIBUSB_ERROR_TIMEOUT => rusb::Error::Timeout,
        LIBUSB_ERROR_OVERFLOW => rusb::Error::Overflow,
        LIBUSB_ERROR_PIPE => rusb::Error::Pipe,
        LIBUSB_ERROR_INTERRUPTED => rusb::Error::Interrupted,
        LIBUSB_ERROR_NO_MEM => rusb::Error::NoMem,
        LIBUSB_ERROR_NOT_SUPPORTED => rusb::Error::NotSupported,
        _ => rusb::Error::Other,
    }
}

#[cfg(test)]
mod tests {
    use crate::usb::DeviceHandle;

    use super::*;

    #[tokio::test]
    async fn submit_completes() {
        let handle: DeviceHandle<rusb::Context> = DeviceHandle::new();
        let buf = UsbMemMut::with_capacity(64);
        let inner_transfer = LibusbTransfer2::new_with_zero_packets();

        let mut transfer = unsafe { inner_transfer.into_int(&handle, 0, buf) };
        let mut cancel = CancellationToken::new();

        let result = tokio::select! {
            result = transfer.submit_and_wait(&mut cancel) => {
                result
            },
            _ = tokio::time::sleep(Duration::from_secs(3)) => {
                panic!("transfer never completed")
            }
        };

        assert_eq!(result, Ok(TransferStatus::Completed));
    }

    #[tokio::test]
    async fn cancel_transfer() {
        let handle: DeviceHandle<rusb::Context> = DeviceHandle::new();
        let buf = UsbMemMut::with_capacity(64);
        let inner_transfer = LibusbTransfer2::new(3);

        let mut transfer = unsafe { inner_transfer.into_int(&handle, 0, buf) };
        let mut cancel = CancellationToken::new();

        let result: Option<_> = tokio::select! {
            result = transfer.submit_and_wait(&mut cancel) => {
                Some(result)
            },
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                None
            }
        };

        match result {
            Some(result) => assert_eq!(result, Ok(TransferStatus::Completed)),
            None => {
                cancel.cancel();
                let result = transfer.wait(&mut cancel).await;
                assert_eq!(result, Ok(TransferStatus::Cancelled));
            }
        }
    }
}
