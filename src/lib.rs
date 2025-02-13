use rusb::constants::{
    LIBUSB_ERROR_ACCESS, LIBUSB_ERROR_BUSY, LIBUSB_ERROR_INTERRUPTED, LIBUSB_ERROR_INVALID_PARAM,
    LIBUSB_ERROR_IO, LIBUSB_ERROR_NOT_FOUND, LIBUSB_ERROR_NOT_SUPPORTED, LIBUSB_ERROR_NO_DEVICE,
    LIBUSB_ERROR_NO_MEM, LIBUSB_ERROR_OTHER, LIBUSB_ERROR_OVERFLOW, LIBUSB_ERROR_PIPE,
    LIBUSB_ERROR_TIMEOUT, LIBUSB_TRANSFER_ADD_ZERO_PACKET, LIBUSB_TRANSFER_CANCELLED,
    LIBUSB_TRANSFER_COMPLETED, LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_NO_DEVICE,
    LIBUSB_TRANSFER_OVERFLOW, LIBUSB_TRANSFER_SHORT_NOT_OK, LIBUSB_TRANSFER_STALL,
    LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_TYPE_BULK, LIBUSB_TRANSFER_TYPE_CONTROL,
    LIBUSB_TRANSFER_TYPE_INTERRUPT, LIBUSB_TRANSFER_TYPE_ISOCHRONOUS,
};
use rusb::ffi::{
    libusb_alloc_transfer, libusb_cancel_transfer, libusb_fill_bulk_transfer,
    libusb_fill_control_transfer, libusb_fill_interrupt_transfer, libusb_fill_iso_transfer,
    libusb_free_transfer, libusb_iso_packet_descriptor, libusb_submit_transfer, libusb_transfer,
};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;
use std::{ffi::c_void, marker::PhantomData, ptr::NonNull};
use tokio::sync::Notify;
use tokio_util::sync::CancellationToken;

use zerocopy_derive::*;

pub use dma::{AllocError, DeviceHandleExt, UsbMemMut};

mod dma;

type Notifier = (Arc<Notify>, Mutex<State>);
type Receiver = Arc<Notify>;

#[repr(transparent)]
struct Ptr<T: ?Sized> {
    pointer: NonNull<T>,
    _p: PhantomData<T>,
}

unsafe impl<T: Send + ?Sized> Send for Ptr<T> {}
unsafe impl<T: Sync + ?Sized> Sync for Ptr<T> {}

impl<T: ?Sized> Ptr<T> {
    pub(crate) fn new(ptr: *mut T) -> Option<Self> {
        if let Some(pointer) = NonNull::new(ptr) {
            Some(Ptr {
                pointer,
                _p: PhantomData,
            })
        } else {
            None
        }
    }

    pub(crate) const fn as_ptr(self) -> *mut T {
        self.pointer.as_ptr()
    }

    pub(crate) const unsafe fn as_ref(&self) -> &T {
        unsafe { self.pointer.as_ref() }
    }

    pub(crate) const unsafe fn as_mut(&mut self) -> &mut T {
        unsafe { self.pointer.as_mut() }
    }

    pub(crate) const fn as_non_null_ptr(self) -> NonNull<T> {
        self.pointer
    }
}

impl<T: Sized> Ptr<T> {
    pub(crate) const fn dangling() -> Self {
        Ptr {
            pointer: NonNull::dangling(),
            _p: PhantomData,
        }
    }
}

impl<T: ?Sized> Clone for Ptr<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: ?Sized> Copy for Ptr<T> {}

impl<T: ?Sized> std::fmt::Debug for Ptr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.pointer.fmt(f)
    }
}

impl<T: ?Sized> std::fmt::Pointer for Ptr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.pointer.fmt(f)
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    #[default]
    Idle,
    Running,
    Ready,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoBytes, FromBytes, Immutable, KnownLayout)]
#[repr(C)]
pub struct ControlPacket {
    pub bm_request_type: u8,
    pub b_request: u8,
    pub w_value: u16,
    pub w_index: u16,
    pub w_length: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoBytes, FromZeros, Unaligned, KnownLayout)]
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

const _: [u8; size_of::<Box<Notifier>>() - size_of::<*mut c_void>()] = [];

#[derive(Debug)]
pub struct InnerTransfer {
    ptr: Ptr<libusb_transfer>,
    num_isos: u16,
    needs_drop: bool,
}

impl InnerTransfer {
    pub fn new(num_isos: usize) -> Self {
        assert!(u16::MAX as usize > num_isos);

        // SAFETY: libusb allocates the data, and returns a null ptr
        //         if the allocation somehow failed.
        let ptr = unsafe { libusb_alloc_transfer(num_isos as i32) };

        unsafe {
            (*ptr).user_data = std::ptr::null_mut();
        }

        Self {
            ptr: Ptr::new(ptr).unwrap(),
            num_isos: num_isos as u16,
            needs_drop: true,
        }
    }

    /// # Safety
    ///
    /// Caller must not call this function if `self`
    /// is owned by a `Transfer`.
    pub(crate) unsafe fn clear(&mut self) {
        let transfer = unsafe { self.as_mut() };
        transfer.dev_handle = std::ptr::null_mut();
        transfer.flags = 0;
        transfer.endpoint = 0;
        transfer.timeout = 0;
        transfer.status = 0;
        transfer.length = 0;
        transfer.actual_length = 0;
        transfer.num_iso_packets = 0;
        transfer.callback = dummy_callback;
        transfer.buffer = std::ptr::null_mut();
        if !transfer.user_data.is_null() {
            let _: Box<Notifier> = Box::from_raw(transfer.user_data.cast());
            transfer.user_data = std::ptr::null_mut();
        }
    }

    pub(crate) const fn as_raw(&self) -> *mut libusb_transfer {
        self.ptr.as_ptr()
    }

    pub(crate) const unsafe fn as_ref(&self) -> &libusb_transfer {
        self.ptr.as_ref()
    }

    pub(crate) const unsafe fn as_mut(&mut self) -> &mut libusb_transfer {
        self.ptr.as_mut()
    }

    pub const fn num_iso_packets(&self) -> usize {
        self.num_isos as usize
    }

    pub(crate) const fn dangling() -> Self {
        Self {
            ptr: Ptr::dangling(),
            num_isos: 0,
            needs_drop: false,
        }
    }

    /// Populates the fields of `self` with the supplied data.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device.
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_interrupt_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_int<C: rusb::UsbContext>(
        self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        mut buf: UsbMemMut,
    ) -> Transfer<C> {
        let notify = Arc::new(Notify::const_new());
        let user_data: *mut Notifier = Box::into_raw(Box::new((notify.clone(), Mutex::default())));

        // Caller ensures that buf.capacity() is the size of the buffer they
        // want passed into the transfer, so we can set the length to 0 to prevent
        // anyone else from obtaining a mutable reference to the buffer data.
        buf.clear();

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_interrupt_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                buf.spare_capacity_mut().len() as i32,
                transfer_callback,
                user_data.cast(),
                u32::MAX,
            )
        };

        Transfer::new(self, notify, buf)
    }

    /// Populates the fields of `self` with the supplied data.
    ///
    /// `timeout` will be truncated to `Duration::from_millis(u32::MAX)`
    /// if greater than `u32::MAX` milliseconds. A timeout of `Duration::ZERO`
    /// indicates no timeout.
    ///
    /// # Safety
    ///
    /// The caller must ensure that:
    ///
    /// - `buf.len() >= 8`
    /// - `ControlPacket::w_length + std::mem::size_of::<ControlPacket>() == buf.capacity()`
    ///
    /// ALSO, the caller must make sure that the first 8 bytes of `buf`
    /// contain the [`ControlPacket`].
    ///
    /// Regardless how much of `buf` contains initialized data, its length will
    /// be set to 0 so that the entire slice can be passed to
    /// `libusb_fill_control_transfer` as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_ctrl<C: rusb::UsbContext>(
        self,
        dev_handle: &rusb::DeviceHandle<C>,
        mut buf: UsbMemMut,
        timeout: Duration,
    ) -> Transfer<C> {
        let notify = Arc::new(Notify::const_new());
        let user_data: *mut Notifier = Box::into_raw(Box::new((notify.clone(), Mutex::default())));
        let timeout = timeout.clamp(Duration::ZERO, Duration::from_millis(u32::MAX as u64));

        // Caller ensures that buf.capacity() is the size of the buffer they
        // want passed into the transfer, so we can set the length to 0 to prevent
        // anyone else from obtaining a mutable reference to the buffer data.
        buf.clear();

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_control_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                transfer_callback,
                user_data.cast(),
                timeout.as_millis() as u32,
            )
        };

        Transfer::new(self, notify, buf)
    }

    /// Populates the fields of `self` with the supplied data.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device.
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_bulk_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_bulk<C: rusb::UsbContext>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        flags: TransferFlags,
        mut buf: UsbMemMut,
    ) -> Transfer<C> {
        let notify = Arc::new(Notify::const_new());
        let user_data: *mut Notifier = Box::into_raw(Box::new((notify.clone(), Mutex::default())));

        buf.clear();

        unsafe {
            libusb_fill_bulk_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                buf.spare_capacity_mut().len() as i32,
                transfer_callback,
                user_data.cast(),
                u32::MAX,
            );

            self.as_mut().flags = flags.bits();
        }

        Transfer::new(self, notify, buf)
    }

    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device. Furthermore,
    /// since this is an isochronous transfer, the contents of `buf` must match
    /// the sections labeled in `iso_packets`.
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_iso_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    ///
    /// # Panics
    ///
    /// This function panics if `iso_packets.len() > self.num_iso_packets()`.
    pub unsafe fn into_iso<C: rusb::UsbContext, T: IsoPacket>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        mut buf: UsbMemMut,
        iso_packets: impl ExactSizeIterator<Item = T>,
    ) -> Transfer<C> {
        let num_iso_packets = iso_packets.len();
        assert!(num_iso_packets <= self.num_iso_packets());
        let notify = Arc::new(Notify::const_new());
        let user_data: *mut Notifier = Box::into_raw(Box::new((notify.clone(), Mutex::default())));

        buf.clear();

        let iso_pkts = std::slice::from_raw_parts_mut(
            self.as_mut().iso_packet_desc.as_mut_ptr(),
            self.as_mut().num_iso_packets as usize,
        );
        iso_packets
            .map(|pkt| pkt.len())
            .zip(iso_pkts)
            .for_each(|(len, pkt)| pkt.length = len);

        unsafe {
            libusb_fill_iso_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                buf.spare_capacity_mut().len() as i32,
                num_iso_packets as i32,
                transfer_callback,
                user_data.cast(),
                u32::MAX,
            );
        }

        Transfer::new(self, notify, buf)
    }
}

unsafe impl Send for InnerTransfer {}

impl Drop for InnerTransfer {
    fn drop(&mut self) {
        // This will only be false when
        // we need a "dangling" version of InnerTransfer
        if self.needs_drop {
            unsafe {
                let user_data: *mut Notifier = self.as_ref().user_data.cast();
                if !user_data.is_null() {
                    _ = Box::from_raw(user_data);
                    self.as_mut().user_data = std::ptr::null_mut();
                }
                self.as_mut().buffer = std::ptr::null_mut();
            }

            // SAFETY: libusb allocated this buffer,
            //         so they get to free it.
            unsafe {
                libusb_free_transfer(self.ptr.as_ptr());
            }
        }
    }
}


/// # Safety
///
/// `Transfer`'s `drop` implementation will panic if the inner transfer is
/// in progress and it cannot grab a handle to the `tokio` runtime. 
/// It needs the runtime so it can continue to wait for the inner transfer 
/// to complete asynchronously, then drop the buffer and inner transfer allocation.
/// 
/// In the above scenario, shutting down the runtime without waiting for the 
/// newly spawned future to complete results in **undefined behavior**, as the
/// transfer callback will most likely write to now-garbage memory.
#[derive(Debug)]
pub struct Transfer<C: rusb::UsbContext> {
    data: Option<(InnerTransfer, Receiver, UsbMemMut)>,
    _ctx: PhantomData<C>,
}

unsafe impl<C: rusb::UsbContext> Send for Transfer<C> {}

impl<C: rusb::UsbContext> Transfer<C> {
    pub(crate) fn new(inner: InnerTransfer, notifier: Receiver, buf: UsbMemMut) -> Self {
        Self {
            data: Some((inner, notifier, buf)),
            _ctx: PhantomData,
        }
    }

    const fn inner(&self) -> &InnerTransfer {
        &self.data.as_ref().unwrap().0
    }

    fn notifier_mut(&mut self) -> &mut Receiver {
        &mut self.data.as_mut().unwrap().1
    }

    fn bytes_mut(&mut self) -> &mut UsbMemMut {
        &mut self.data.as_mut().unwrap().2
    }

    fn state(&self) -> State {
        // SAFETY: Both pointers are always valid
        //         and no one takes a mutable reference
        //         to them.
        unsafe {
            *self
                .inner()
                .as_ref()
                .user_data
                .cast::<Notifier>()
                .as_ref()
                .unwrap()
                .1
                .lock()
                .unwrap()
        }
    }

    fn state_mut(&mut self) -> MutexGuard<'_, State> {
        // SAFETY: Both pointers are always valid
        //         and no one takes a mutable reference
        //         to them.
        unsafe {
            self.inner()
                .as_ref()
                .user_data
                .cast::<Notifier>()
                .as_ref()
                .unwrap()
                .1
                .lock()
                .unwrap()
        }
    }

    /// Submits this transfer to libusb.
    ///
    /// # Errors
    ///
    /// If this function encounters any error while fulfilling the transfer
    /// request, an error variant will be returned. If this function returns an
    /// error, no data was transferred.
    ///
    /// The errors returned by this function include:
    ///
    /// - `NoDevice` if the device has been disconnected
    /// - `InvalidParam` if the transfer size is larger than the operating
    ///   system and/or hardware can support
    /// - `Busy` if this is a resubmit and the buffer hadn't been reset
    /// - `Error` on general failure
    ///
    /// # Safety
    /// 
    /// The caller must not drop the resulting future until completion.
    ///
    /// Dropping future and the tranfer does not immediately result in
    /// undefined behavior. See struct definition for more info.
    ///
    /// # Cancel safety
    ///
    /// If `submit` is used as an event in [`tokio::select`] and some other branch
    /// completes first, then future calls to `submit` will continue waiting for the
    /// transfer to complete.
    ///
    /// If [`Transfer`] is dropped after cancelling the future,
    /// it will send a cancellation request to libusb and one of three things will happen:
    ///
    /// 1. If the transfer is complete, then all parts of the transfer are 
    ///    dropped/deallocated safely.
    /// 2. If the transfer is not complete and the function can access the tokio runtime,
    ///    then the function spawns a new future into the executor to finish polling for
    ///    the transfer callback, then drops the rest of the transfer.
    /// 3. If the transfer is not complete and the function can't access the tokio runtime,
    ///    then the function panics. (TODO: Not a great alternative)
    #[must_use = "futures do nothing unless polled"]
    pub async unsafe fn submit(
        &mut self,
        cancel_token: CancellationToken,
    ) -> Result<TransferStatus, rusb::Error> {
        if State::Ready == self.state() {
            // SAFETY: Transfer is ready, therefore libusb is no longer mutating `status`
            let status = TransferStatus::from_i32(unsafe { self.inner().as_ref().status }).unwrap();
            return Ok(status);
        }

        // SAFETY: We hold exclusive mutable access to `self`.
        //         Therefore, we can prevent anyone else from
        //         accessing the UsbMemMut and the inner transfer
        //         while libusb uses the data.
        match unsafe { self.inner_submit() } {
            Ok(_) => {
                let mut state = self.state_mut();
                if State::Ready != *state {
                    *state = State::Running;
                }
            }
            Err(rusb::Error::Busy) => (),
            Err(err) => return Err(err),
        }

        enum Event {
            Cancelled,
            Completed,
        }

        if let Event::Cancelled = tokio::select! {
            biased;
            _ = self.notifier_mut().notified() => Event::Completed,
            _ = cancel_token.cancelled() => Event::Cancelled,
        } {
            _ = self.cancel();
            self.notifier_mut().notified().await;
        };

        // SAFETY: libusb has completed the transfer and is no
        //         longer using the transfer struct, making it
        //         safe for us to use.
        //         Furthermore, transfer is a well-aligned and valid
        //         pointer, making it safe to access.
        unsafe {
            // Update UsbMemMut with the new buffer length from transfer
            let transfer_ref = self.inner().as_ref();
            let mut transfer_len = transfer_ref.actual_length as usize;
            if LIBUSB_TRANSFER_TYPE_CONTROL == transfer_ref.transfer_type {
                transfer_len += size_of::<ControlPacket>();
            }
            // Get status from transfer
            let status = TransferStatus::from_i32(transfer_ref.status).unwrap();

            self.bytes_mut().set_len(transfer_len);

            Ok(status)
        }
    }

    /// # Safety
    ///
    /// Caller must make sure that the transfer's buffer contains
    /// the right data for the transfer type.
    unsafe fn inner_submit(&mut self) -> Result<(), rusb::Error> {
        // SAFETY: `self.inner` is a valid pointer for the entirety
        //         of self's lifetime. Also, caller ensures that
        //         the transfer buffer points to the right data for
        //         this transfer type.
        let result = unsafe { libusb_submit_transfer(self.inner().as_raw()) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    fn cancel(&mut self) -> Result<(), rusb::Error> {
        // SAFETY: `self.inner` is a valid pointer
        //         for the entirety of self's lifetime.
        let result = unsafe { libusb_cancel_transfer(self.inner().as_raw()) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    /// Returns the transfer's timeout. A value
    /// of `Duration::ZERO` indicates no timeout.
    pub const fn timeout(&self) -> Duration {
        // SAFETY: `self.inner` is a valid and initialized
        //         pointer to a `libusb_transfer`.
        unsafe { Duration::from_millis(self.inner().as_ref().timeout as u64) }
    }

    /// Returns the transfer's `rusb::TransferType`.
    pub const fn kind(&self) -> rusb::TransferType {
        unsafe {
            match self.inner().as_ref().transfer_type {
                LIBUSB_TRANSFER_TYPE_CONTROL => rusb::TransferType::Control,
                LIBUSB_TRANSFER_TYPE_INTERRUPT => rusb::TransferType::Interrupt,
                LIBUSB_TRANSFER_TYPE_BULK => rusb::TransferType::Bulk,
                LIBUSB_TRANSFER_TYPE_ISOCHRONOUS => rusb::TransferType::Isochronous,
                _ => unreachable!(),
            }
        }
    }

    /// Returns a slice containing the transfer's initialized buffer.
    /// During a call to [`Transfer::submit`], this function returns
    /// `None`.
    pub fn buf(&self) -> Option<&[u8]> {
        if State::Running == self.state() {
            None
        } else {
            Some(self.bytes())
        }
    }

    /// Returns a mutable slice containing the transfer's initialized
    /// buffer. During a call to [`Transfer::submit`], this function returns
    /// `None`.
    pub fn buf_mut(&mut self) -> Option<&mut [u8]> {
        if State::Running == self.state() {
            None
        } else {
            Some(self.bytes_mut())
        }
    }

    /// Returns `None` if the transfer type is not isochronous or if
    /// the transfer is not ready.
    pub fn iso_packets(&self) -> Option<&[impl IsoPacket]> {
        if State::Running == self.state() || rusb::TransferType::Isochronous != self.kind() {
            None
        } else {
            // SAFETY: `num_iso_packets` is never mutated
            //         and therefore correctly describes
            //         the length of this slice. Also,
            //         we guarantee through the state that
            //         we are the only place referencing this data.
            let packets = unsafe {
                std::slice::from_raw_parts(
                    self.inner().as_ref().iso_packet_desc.as_ptr(),
                    self.inner().as_ref().num_iso_packets as usize,
                )
            };
            Some(packets)
        }
    }

    pub fn into_buf(mut self) -> Option<UsbMemMut> {
        if State::Running == self.state() {
            None
        } else {
            self.data.take().map(|(_, _, bytes_mut)| bytes_mut)
        }
    }

    pub fn into_parts(mut self) -> Option<(InnerTransfer, UsbMemMut)> {
        if State::Running == self.state() {
            None
        } else {
            let (mut inner, _, buf) = self.data.take().unwrap();
            // SAFETY: Inner transfer is no longer owned by `Transfer`.
            unsafe { inner.clear() };
            Some((inner, buf))
        }
    }

    fn bytes(&self) -> &UsbMemMut {
        &self.data.as_ref().unwrap().2
    }
}

impl<C: rusb::UsbContext> Drop for Transfer<C> {
    fn drop(&mut self) {
        if self
            .data
            .as_ref()
            .is_some_and(|_| State::Running == self.state())
        {
            _ = self.cancel();
            let handle = tokio::runtime::Handle::current();
            let (inner, notifier, buf) = self.data.take().unwrap();
            handle.spawn(async move {
                notifier.notified().await;
                let _ = inner;
                let _ = buf;
            });
        }
    }
}

/// Handles a USB transfer completion, cancellation, error, or whatever,
/// by simply alerting the function that submitted the transfer.
extern "system" fn transfer_callback(transfer: *mut libusb_transfer) {
    // SAFETY: `transfer` is a valid pointer and can be used to access
    //         `user_data`. `user_data` is a valid `Box<Notifier>` pointer
    //         created from `Box::into_raw`, and can be turned back into
    //         a box.
    let (notifier, state) = unsafe {
        let user_data: *const Notifier = transfer.as_ref().unwrap().user_data.cast();
        user_data.as_ref().unwrap()
    };

    // Potential race condition here:
    //
    // After notifying the caller, caller can lock the mutex before we get
    // a chance to lock the mutex, leading to weird stuff happening.
    // Therefore we hold the lock while sending our signal so that the next
    // thread that views the state will see the correct state.
    let mut state = state.lock().unwrap();
    notifier.notify_one();
    *state = State::Ready;
}

extern "system" fn dummy_callback(_: *mut libusb_transfer) {}

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
        LIBUSB_ERROR_OTHER | _ => rusb::Error::Other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
