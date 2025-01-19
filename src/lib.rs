use bytes::BytesMut;
use rusb::constants::{
    LIBUSB_ERROR_ACCESS, LIBUSB_ERROR_BUSY, LIBUSB_ERROR_INTERRUPTED, LIBUSB_ERROR_INVALID_PARAM,
    LIBUSB_ERROR_IO, LIBUSB_ERROR_NOT_FOUND, LIBUSB_ERROR_NOT_SUPPORTED, LIBUSB_ERROR_NO_DEVICE,
    LIBUSB_ERROR_NO_MEM, LIBUSB_ERROR_OTHER, LIBUSB_ERROR_OVERFLOW, LIBUSB_ERROR_PIPE,
    LIBUSB_ERROR_TIMEOUT, LIBUSB_TRANSFER_CANCELLED, LIBUSB_TRANSFER_COMPLETED,
    LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_NO_DEVICE, LIBUSB_TRANSFER_OVERFLOW,
    LIBUSB_TRANSFER_STALL, LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_TYPE_BULK,
    LIBUSB_TRANSFER_TYPE_CONTROL, LIBUSB_TRANSFER_TYPE_INTERRUPT, LIBUSB_TRANSFER_TYPE_ISOCHRONOUS,
};
use rusb::ffi::{
    libusb_alloc_transfer, libusb_cancel_transfer, libusb_fill_bulk_transfer,
    libusb_fill_control_transfer, libusb_fill_interrupt_transfer, libusb_fill_iso_transfer,
    libusb_free_transfer, libusb_iso_packet_descriptor, libusb_submit_transfer, libusb_transfer,
};
use std::sync::{Mutex, MutexGuard};
use std::time::Duration;
use std::{ffi::c_void, marker::PhantomData, ptr::NonNull};
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

use zerocopy_derive::*;

mod dma;

type Notifier = (mpsc::Sender<()>, Mutex<State>);
type Receiver = mpsc::Receiver<()>;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, IntoBytes, FromZeros)]
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
        mut buf: BytesMut,
    ) -> Transfer<C> {
        debug_assert_eq!(size_of::<Box<Notifier>>(), size_of::<*mut c_void>());
        let (tx, rx) = mpsc::channel(1);
        let user_data: *mut Notifier = Box::into_raw(Box::new((tx, Mutex::default())));

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

        Transfer::new(self, rx, buf)
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
    /// `libusb_fill_interrupt_transfer` as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_ctrl<C: rusb::UsbContext>(
        self,
        dev_handle: &rusb::DeviceHandle<C>,
        mut buf: BytesMut,
        timeout: Duration,
    ) -> Transfer<C> {
        debug_assert_eq!(size_of::<Box<Notifier>>(), size_of::<*mut c_void>());
        let (tx, rx) = mpsc::channel(1);
        let user_data: *mut Notifier = Box::into_raw(Box::new((tx, Mutex::default())));
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

        Transfer::new(self, rx, buf)
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
    pub unsafe fn into_bulk<C: rusb::UsbContext>(
        self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        mut buf: BytesMut,
    ) -> Transfer<C> {
        debug_assert_eq!(size_of::<Box<Notifier>>(), size_of::<*mut c_void>());
        let (tx, rx) = mpsc::channel(1);
        let user_data: *mut Notifier = Box::into_raw(Box::new((tx, Mutex::default())));

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
        }

        Transfer::new(self, rx, buf)
    }

    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device. Furthermore,
    /// since this is an isochronous transfer, the contents of `buf` must match
    /// the sections labeled in `iso_packets`.
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_interrupt_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    ///
    /// # Panics
    ///
    /// This function panics if `iso_packets.len() > self.num_iso_packets()`.
    pub unsafe fn into_iso<C: rusb::UsbContext, T: IsoPacket>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        mut buf: BytesMut,
        iso_packets: impl ExactSizeIterator<Item = T>,
    ) -> Transfer<C> {
        let num_iso_packets = iso_packets.len();
        assert!(num_iso_packets <= self.num_iso_packets());
        debug_assert_eq!(size_of::<Box<Notifier>>(), size_of::<*mut c_void>());
        let (tx, rx) = mpsc::channel(1);
        let user_data: *mut Notifier = Box::into_raw(Box::new((tx, Mutex::default())));

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

        Transfer::new(self, rx, buf)
    }
}

impl Drop for InnerTransfer {
    fn drop(&mut self) {
        // The only time the transfer needs to call its drop
        // impl is when it's not owned by any other struct.
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

#[derive(Debug)]
pub struct Transfer<C: rusb::UsbContext> {
    inner: InnerTransfer,
    notifier: Receiver,
    buf: BytesMut,
    _ctx: PhantomData<C>,
}

unsafe impl<C: rusb::UsbContext> Send for Transfer<C> {}

impl<C: rusb::UsbContext> Transfer<C> {
    pub(crate) fn new(inner: InnerTransfer, notifier: Receiver, buf: BytesMut) -> Self {
        Self {
            inner,
            notifier,
            buf,
            _ctx: PhantomData,
        }
    }

    fn state(&self) -> State {
        // SAFETY: Both pointers are always valid
        //         and no one takes a mutable reference
        //         to them.
        unsafe {
            *self
                .inner
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
            self.inner
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
    /// # Cancel safety
    ///
    /// If `submit` is used as an event in [`tokio::select`] and some other branch
    /// completes first, then future calls to `submit` will continue waiting for the
    /// transfer to complete.
    ///
    /// If [`Transfer`] is dropped after cancelling the future,
    /// it will send a cancellation request to libusb and block until the transfer
    /// completes, then deallocates the transfer to prevent leaking memory/undefined behavior.
    pub async fn submit(
        &mut self,
        cancel_token: CancellationToken,
    ) -> Result<TransferStatus, rusb::Error> {
        if State::Ready == self.state() {
            // SAFETY: Transfer is ready, therefore libusb is no longer mutating `status`
            let status = TransferStatus::from_i32(unsafe { self.inner.as_ref().status }).unwrap();
            return Ok(status);
        }

        // SAFETY: We hold exclusive mutable access to `self`.
        //         Therefore, we can prevent anyone else from
        //         accessing the BytesMut and the inner transfer
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
            _ = self.notifier.recv() => Event::Completed,
            _ = cancel_token.cancelled() => Event::Cancelled,
        } {
            if self.cancel().is_ok() {
                self.notifier.recv().await.unwrap();
            }
        };

        // SAFETY: libusb has completed the transfer and is no
        //         longer using the transfer struct, making it
        //         safe for us to use.
        //         Furthermore, transfer is a well-aligned and valid
        //         pointer, making it safe to access.
        unsafe {
            // Update BytesMut with the new buffer length from transfer
            let transfer_ref = self.inner.as_ref();
            let mut transfer_len = transfer_ref.actual_length as usize;
            if LIBUSB_TRANSFER_TYPE_CONTROL == transfer_ref.transfer_type {
                transfer_len += size_of::<ControlPacket>();
            }
            self.buf.set_len(transfer_len);

            // Get status from transfer
            let status = TransferStatus::from_i32(transfer_ref.status).unwrap();

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
        let result = unsafe { libusb_submit_transfer(self.inner.as_raw()) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    fn cancel(&mut self) -> Result<(), rusb::Error> {
        // SAFETY: `self.inner` is a valid pointer
        //         for the entirety of self's lifetime.
        let result = unsafe { libusb_cancel_transfer(self.inner.as_raw()) };
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
        unsafe { Duration::from_millis(self.inner.as_ref().timeout as u64) }
    }

    /// Returns the transfer's `rusb::TransferType`.
    pub const fn kind(&self) -> rusb::TransferType {
        unsafe {
            match self.inner.as_ref().transfer_type {
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
            Some(&self.buf)
        }
    }

    /// Returns a mutable slice containing the transfer's initialized
    /// buffer. During a call to [`Transfer::submit`], this function returns
    /// `None`.
    pub fn buf_mut(&mut self) -> Option<&mut [u8]> {
        if State::Running == self.state() {
            None
        } else {
            Some(&mut self.buf)
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
                    self.inner.as_ref().iso_packet_desc.as_ptr(),
                    self.inner.as_ref().num_iso_packets as usize,
                )
            };
            Some(packets)
        }
    }

    pub fn into_buf(mut self) -> Option<BytesMut> {
        if State::Running == self.state() {
            None
        } else {
            // Transfer is ready, therefore we can replace
            // self.buf with an empty BytesMut.
            let buf = std::mem::replace(&mut self.buf, BytesMut::new());
            Some(buf)
        }
    }

    pub fn into_parts(mut self) -> Option<(InnerTransfer, BytesMut)> {
        if State::Running == self.state() {
            None
        } else {
            let mut inner = std::mem::replace(&mut self.inner, InnerTransfer::dangling());
            // SAFETY: Inner transfer is no longer owned by `Transfer`.
            unsafe { inner.clear() };
            let buf = std::mem::replace(&mut self.buf, BytesMut::new());
            Some((inner, buf))
        }
    }
}

impl<C: rusb::UsbContext> Drop for Transfer<C> {
    fn drop(&mut self) {
        match self.state() {
            State::Running => {
                _ = self.cancel();
                if let Ok(handle) = tokio::runtime::Handle::try_current() {
                    handle.block_on(self.notifier.recv());
                } else {
                    _ = self.notifier.blocking_recv();
                }
            }
            State::Idle | State::Ready => (),
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
    let notifier = unsafe {
        let user_data: *const Notifier = transfer.as_ref().unwrap().user_data.cast();
        user_data.as_ref().unwrap()
    };

    _ = notifier.0.blocking_send(());
    *notifier.1.lock().unwrap() = State::Ready;
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

    #[test]
    fn transfer_is_send() {
        fn send_thing<T: Send>(_item: T) {}

        let handle = rusb::open_device_with_vid_pid(0x0000, 0x0000).unwrap();
        let transfer = InnerTransfer::new(0);
        let transfer = unsafe {
            transfer.into_ctrl(&handle, BytesMut::with_capacity(64), Duration::from_secs(5))
        };

        send_thing(transfer);
    }
}
