use completion::{Completion, Event};
use pin_project_lite::pin_project;
use ptr::Ptr;
use rusb::constants::{
    LIBUSB_ERROR_ACCESS, LIBUSB_ERROR_BUSY, LIBUSB_ERROR_INTERRUPTED, LIBUSB_ERROR_INVALID_PARAM,
    LIBUSB_ERROR_IO, LIBUSB_ERROR_NO_DEVICE, LIBUSB_ERROR_NO_MEM, LIBUSB_ERROR_NOT_FOUND,
    LIBUSB_ERROR_NOT_SUPPORTED, LIBUSB_ERROR_OVERFLOW, LIBUSB_ERROR_PIPE,
    LIBUSB_ERROR_TIMEOUT, LIBUSB_TRANSFER_ADD_ZERO_PACKET, LIBUSB_TRANSFER_CANCELLED,
    LIBUSB_TRANSFER_COMPLETED, LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_NO_DEVICE,
    LIBUSB_TRANSFER_OVERFLOW, LIBUSB_TRANSFER_SHORT_NOT_OK, LIBUSB_TRANSFER_STALL,
    LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_TYPE_BULK, LIBUSB_TRANSFER_TYPE_CONTROL,
    LIBUSB_TRANSFER_TYPE_INTERRUPT, LIBUSB_TRANSFER_TYPE_ISOCHRONOUS,
};
use rusb::ffi::{
    libusb_fill_bulk_transfer, libusb_fill_control_transfer, libusb_fill_interrupt_transfer,
    libusb_fill_iso_transfer, libusb_iso_packet_descriptor, libusb_transfer,
};
use std::cell::UnsafeCell;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::task::{Poll, ready};
use std::time::Duration;
use std::{ffi::c_void, marker::PhantomData};
use tokio::sync::Notify;
use tokio_util::sync::CancellationToken;
use usb::{
    libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer, libusb_submit_transfer,
};

#[cfg(feature = "zerocopy")]
use zerocopy_derive::*;

pub use dma::{AllocError, DeviceHandleExt};
pub use usb::UsbMemMut;

mod completion;
mod dma;
mod ptr;
mod state;
mod usb;

/// Data we share between the libusb transfer
/// and our transfer struct.
///
/// We need a way to give out:
/// - Two references to a notifier,
/// - A (probably unsafe) mutable reference to a buffer,
/// - Two mutable references to a State object of some sorts,
///
/// Since this will be wrapped in an Arc, we can
/// give out references to a notifier easily.
///
/// We need our struct to be Send and Sync. UnsafeCell won't let
/// us do this, but the "state" variable will ensure we never
/// have a data race. So we create a new container called "UnsaferCell",
/// which is Sync and will let us do our thing.
#[derive(Debug)]
struct UserData {
    completion: Notify,
    state: state::State,
    buf: UnsaferCell<UsbMemMut>,
}

impl UserData {
    pub(crate) fn notify_ready(&self) {
        // Set the state before sending the
        // notification so that we don't panic
        // when trying to access the shared data
        // later.
        self.state.set_ready();
        self.completion.notify_one();
    }

    const fn as_refs(&self) -> (&Notify, &state::State, &UnsaferCell<UsbMemMut>) {
        let UserData {
            completion,
            state,
            buf,
        } = self;
        (completion, state, buf)
    }
}

/// A wrapper around [`UnsafeCell`] that
/// implements [`Sync`]. Extremely unsafe to
/// use, because now there is nothing
/// to prevent sharing a value in here across
/// multiple threads.
///
/// This library uses this to give partial ownership
/// of a buffer to a transfer owned by libusb. While
/// the transfer is active no one is allowed to even
/// read this buffer, since libusb and the OS are busy
/// reading/writing to it.
#[derive(Debug)]
#[repr(transparent)]
struct UnsaferCell<T>(pub UnsafeCell<T>);
unsafe impl<T> Sync for UnsaferCell<T> {}

const fn _is_send_and_sync<T: Send + Sync>() {}
const _: () = _is_send_and_sync::<UserData>();

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

// Ensure that our user data isn't a wide pointer and can be casted to
// a void pointer safely.
const _: [u8; size_of::<Arc<UserData>>() - size_of::<*mut c_void>()] = [];

#[derive(Debug)]
pub struct InnerTransfer {
    ptr: Ptr<libusb_transfer>,
    num_isos: u16,
}

impl InnerTransfer {
    /// Allocates a new `InnerTransfer` on the heap with room for
    /// `num_iso` packets.
    ///
    /// # Panic
    ///
    /// Panics if `num_isos > u16::MAX`.
    pub fn new(num_isos: usize) -> Self {
        assert!(u16::MAX as usize >= num_isos);

        // SAFETY: libusb allocates the data, and returns a null ptr
        //         if the allocation somehow failed.
        let ptr = unsafe { libusb_alloc_transfer(num_isos as i32) };

        unsafe {
            (*ptr).user_data = std::ptr::null_mut();
        }

        Self {
            ptr: Ptr::new(ptr).expect("libusb allocation shouldn't have failed"),
            num_isos: num_isos as u16,
        }
    }

    /// Clears all data out of this inner transfer,
    /// setting all pointers to `std::ptr::null_mut()`.
    ///
    /// If `user_data` is set, then this function drops
    /// the data. Currently `user_data` is an `Arc<UserData>`,
    /// so this function decrements the reference count.
    ///
    /// # Safety
    ///
    /// It is undefined behavior to call this function during
    /// an active transfer as the libusb runtime could access
    /// any of this data at the same time, potentially on another
    /// thread.
    ///
    /// Additionally, caller must not call this function if `self`
    /// is part of a [`Transfer`] struct at all, as submitting
    /// a transfer to libusb could do bad things if `self` had
    /// been previously cleared out.
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
        // SAFETY: Caller upholds contract that we're not
        // in a transfer.
        unsafe { self.take_user_data() };
    }

    /// Takes the user data out of the `self`, leaving a null pointer
    /// in its place.
    ///
    /// # Safety
    ///
    /// It is undefined behavior to call this during an active transfer
    /// as the libusb runtime could access this data at the same time,
    /// maybe even on another thread.
    pub(crate) unsafe fn take_user_data(&mut self) -> Option<Arc<UserData>> {
        let transfer = unsafe { self.as_mut() };
        if !transfer.user_data.is_null() {
            let user_data: Arc<UserData> = unsafe { Arc::from_raw(transfer.user_data.cast()) };
            transfer.user_data = std::ptr::null_mut();
            Some(user_data)
        } else {
            None
        }
    }

    /// Returns a raw pointer to the allocated `libusb_transfer` struct.
    #[inline]
    pub(crate) const fn as_raw(&self) -> *mut libusb_transfer {
        self.ptr.as_ptr()
    }

    #[inline]
    pub(crate) const unsafe fn as_ref(&self) -> &libusb_transfer {
        unsafe { self.ptr.as_ref() }
    }

    #[inline]
    pub(crate) const unsafe fn as_mut(&mut self) -> &mut libusb_transfer {
        unsafe { self.ptr.as_mut() }
    }

    #[inline]
    pub const fn num_iso_packets(&self) -> usize {
        self.num_isos as usize
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity, *not the length*, of `buf`
    /// is exactly the size of the buffer they want passed to the USB device.
    ///
    /// Regardless how much of `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_interrupt_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_int<C: rusb::UsbContext>(
        self,
        dev_handle: &usb::DeviceHandle<C>,
        endpoint: u8,
        buf: UsbMemMut,
    ) -> Transfer<C> {
        let (ptr, len, user_data) = make_user_data(buf);

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_interrupt_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                ptr,
                len,
                transfer_callback,
                Arc::into_raw(Arc::clone(&user_data)).cast_mut().cast(),
                u32::MAX,
            )
        };

        Transfer::new(self, user_data)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
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
        buf: UsbMemMut,
        timeout: Duration,
    ) -> Transfer<C> {
        let timeout = timeout.clamp(Duration::ZERO, Duration::from_millis(u32::MAX as u64));
        let (ptr, _, user_data) = make_user_data(buf);

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_control_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                ptr,
                transfer_callback,
                Arc::into_raw(Arc::clone(&user_data)).cast_mut().cast(),
                timeout.as_millis() as u32,
            )
        };

        Transfer::new(self, user_data)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity, *not the length*, of `buf`
    /// is exactly the size of the buffer they want passed to the USB device.
    ///
    /// Regardless how much of `buf` contains initialized data, its length
    /// will be set to 0 so that the entire slice can be passed to
    /// `libusb_fill_bulk_transfer` as a `*mut MaybeUninit<u8>`.
    pub unsafe fn into_bulk<C: rusb::UsbContext>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        flags: TransferFlags,
        buf: UsbMemMut,
    ) -> Transfer<C> {
        let (ptr, len, user_data) = make_user_data(buf);

        unsafe {
            libusb_fill_bulk_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                ptr,
                len,
                transfer_callback,
                Arc::into_raw(Arc::clone(&user_data)).cast_mut().cast(),
                u32::MAX,
            );

            self.as_mut().flags = flags.bits();
        }

        Transfer::new(self, user_data)
    }

    /// Populates the fields of `self` with the supplied data and returns
    /// a new [`Transfer`] ready to submit.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity, *not the length*, of `buf`
    /// is exactly the size of the buffer they want passed to the USB device.
    /// Furthermore, since this is an isochronous transfer, the contents of
    /// `buf` must match the sections labeled in `iso_packets`.
    ///
    /// Regardless how much of `buf` contains initialized data, its length
    /// will be set to 0 so that the entire slice can be passed to
    /// `libusb_fill_iso_transfer` as a `*mut MaybeUninit<u8>`.
    ///
    /// # Panics
    ///
    /// This function panics if `iso_packets.len() > self.num_iso_packets()`.
    pub unsafe fn into_iso<C: rusb::UsbContext, T: IsoPacket>(
        mut self,
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        buf: UsbMemMut,
        iso_packets: impl ExactSizeIterator<Item = T>,
    ) -> Transfer<C> {
        let num_iso_packets = iso_packets.len();
        assert!(num_iso_packets <= self.num_iso_packets());
        let (ptr, len, user_data) = make_user_data(buf);

        // SAFETY: `num_iso_packets` contains the length of
        // this slice of packet descriptors, and the mutable
        // pointer comes from an owned allocation.
        let iso_pkts = unsafe {
            std::slice::from_raw_parts_mut(
                self.as_mut().iso_packet_desc.as_mut_ptr(),
                self.as_mut().num_iso_packets as usize,
            )
        };
        iso_packets
            .map(|pkt| pkt.len())
            .zip(iso_pkts)
            .for_each(|(len, pkt)| pkt.length = len);

        unsafe {
            libusb_fill_iso_transfer(
                self.as_raw(),
                dev_handle.as_raw(),
                endpoint,
                ptr,
                len,
                num_iso_packets as i32,
                transfer_callback,
                Arc::into_raw(Arc::clone(&user_data)).cast_mut().cast(),
                u32::MAX,
            );
        }

        Transfer::new(self, user_data)
    }
}

unsafe impl Send for InnerTransfer {}

impl Drop for InnerTransfer {
    fn drop(&mut self) {
        // SAFETY: We never drop during an active transfer.
        unsafe { self.clear() };

        // SAFETY: libusb allocated this buffer,
        //         so they get to free it.
        unsafe {
            libusb_free_transfer(self.ptr.as_ptr());
        }
    }
}

#[inline]
fn make_user_data(mut buf: UsbMemMut) -> (*mut u8, i32, Arc<UserData>) {
    buf.truncate(0);
    let completion = Notify::const_new();
    let state = state::State::new_idle();
    let (ptr, len) = {
        let spare = buf.spare_capacity_mut();
        (spare.as_mut_ptr().cast(), spare.len() as i32)
    };
    let user_data = Arc::new(UserData {
        completion,
        state,
        buf: UnsaferCell(UnsafeCell::new(buf)),
    });

    (ptr, len, user_data)
}

#[derive(Debug)]
enum FutureState {
    Init,
    InFlight,
    Cancelled,
    Completed,
}

pin_project! {
    #[project = TransferFutureProj]
    pub struct TransferFuture<'a> {
        #[pin]
        completion: Completion<'a>,
        fut_state: FutureState,
        transfer_state: &'a state::State,
        transfer: *mut libusb_transfer,
        buf: &'a UnsaferCell<UsbMemMut>,
    }
}

impl TransferFutureProj<'_, '_> {
    /// # Safety
    ///
    /// Caller must make sure that the transfer's buffer contains
    /// the right data for the transfer type.
    unsafe fn submit(&self) -> rusb::Result<()> {
        // SAFETY: `self.transfer` is a valid pointer for the entirety
        //         of self's lifetime. Also, caller ensures that
        //         the transfer buffer points to the right data for
        //         this transfer type.
        let result = unsafe { libusb_submit_transfer(*self.transfer) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    fn cancel(&self) -> rusb::Result<()> {
        // SAFETY: `self.transfer` is a valid pointer
        //         for the entirety of self's lifetime.
        let result = unsafe { libusb_cancel_transfer(*self.transfer) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    #[inline]
    const fn state(&self) -> &state::State {
        self.transfer_state
    }

    /// # Safety
    ///
    /// It is undefined behaviour to call this while
    /// a transfer is active, since that would break
    /// the aliasing rule.
    #[inline]
    unsafe fn buf(&mut self) -> &mut UsbMemMut {
        // SAFETY: Caller ensures that we don't access
        // this memory while a transfer is active.
        unsafe { self.buf.0.get().as_mut().unwrap() }
    }
}

impl Future for TransferFuture<'_> {
    type Output = rusb::Result<TransferStatus>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let mut this = self.as_mut().project();
        loop {
            match *this.fut_state {
                FutureState::Init => {
                    // Submit was just called and we just started polling this transfer,
                    // so we need to check that this transfer wasn't previously submitted.
                    // Otherwise, we submit the transfer and transition to SubmitState::InFlight.
                    *this.fut_state = match this.state().get() {
                        state::StateRepr::Idle => {
                            // SAFETY: Creator of this transfer upholds safety contract.
                            match unsafe { this.submit() } {
                                Ok(_) => {
                                    this.state().set_running();
                                }
                                Err(rusb::Error::Busy) => (),
                                Err(err) => break Poll::Ready(Err(err)),
                            }
                            FutureState::InFlight
                        }
                        state::StateRepr::Running => FutureState::InFlight,
                        state::StateRepr::Ready => FutureState::Completed,
                    }
                }
                FutureState::InFlight | FutureState::Cancelled => {
                    // The transfer is active so we need to poll for both the completion
                    // notification and a cancellation. If we get a cancel, then we transition
                    // to SubmitState::Cancelled and poll for the completion, otherwise we go to
                    // SubmitState::Completed.
                    let event = ready!(this.completion.as_mut().poll(cx));
                    *this.fut_state = match event {
                        Event::Completed => FutureState::Completed,
                        Event::Cancelled => {
                            _ = this.cancel();
                            FutureState::Cancelled
                        }
                    };
                }
                FutureState::Completed => {
                    // SAFETY: Transfer is done, we're free to access the data!
                    unsafe {
                        let transfer_ref = this.transfer.as_ref().unwrap();
                        let mut transfer_len = transfer_ref.actual_length as usize;
                        if LIBUSB_TRANSFER_TYPE_CONTROL == transfer_ref.transfer_type {
                            transfer_len += size_of::<ControlPacket>();
                        }

                        let status = TransferStatus::from_i32(transfer_ref.status).unwrap();

                        // Update UsbMemMut with the new buffer length
                        // SAFETY: This is the only place we access/mutate UsbMemMut.
                        this.buf().set_len(transfer_len);

                        break Poll::Ready(Ok(status));
                    }
                }
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
    inner: Option<InnerTransfer>,
    user_data: Option<Arc<UserData>>,
    _ctx: PhantomData<C>,
}

unsafe impl<C: rusb::UsbContext> Send for Transfer<C> {}

impl<C: rusb::UsbContext> Transfer<C> {
    pub(crate) const fn new(inner: InnerTransfer, user_data: Arc<UserData>) -> Self {
        Self {
            inner: Some(inner),
            user_data: Some(user_data),
            _ctx: PhantomData,
        }
    }

    #[inline]
    fn do_if_idle<T>(&self, f: impl FnOnce(&Self) -> T) -> Option<T> {
        if state::StateRepr::Running == self.state().get() {
            None
        } else {
            Some(f(self))
        }
    }

    #[inline]
    fn do_if_idle_mut<'a, T>(&'a mut self, f: impl FnOnce(&'a mut Self) -> T) -> Option<T> {
        if state::StateRepr::Running == self.state().get() {
            None
        } else {
            Some(f(self))
        }
    }

    #[inline]
    const fn inner(&self) -> &InnerTransfer {
        self.inner.as_ref().unwrap()
    }

    #[inline]
    fn state(&self) -> &state::State {
        &self.user_data.as_ref().unwrap().state
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
    /// This function assumes that the caller upheld the contract specified
    /// by the `into_iso`, `into_int`, `into_ctrl`, `into_bulk` functions.
    /// Otherwise, the transfer write into uninitialized memory.
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
    /// 2. If the transfer is not complete and the function can access the async runtime,
    ///    then the function spawns a new future into the executor to finish polling for
    ///    the transfer callback, then drops the rest of the transfer.
    /// 3. If the transfer is not complete and the function can't access the async runtime,
    ///    then the function leaks the buffer and transfer to prevent the callback
    ///    function from using freed memory.
    ///
    /// Scenario 3 has some implications if only using a portion of the DMA mapping, since
    /// [`UsbMemMut`] requires a unique reference to the mapping to reclaim any part of the
    /// allocation. TODO.
    #[must_use = "futures do nothing unless polled"]
    pub fn submit<'a>(&'a self, cancel_token: &'a CancellationToken) -> TransferFuture<'a> {
        let transfer = self.inner().as_raw();
        let (completion, state, buf) = self.user_data.as_ref().unwrap().as_refs();
        TransferFuture {
            completion: Completion::new(completion, cancel_token),
            fut_state: FutureState::Init,
            transfer_state: state,
            transfer,
            buf,
        }
    }

    #[inline]
    pub fn is_running(&self) -> bool {
        self.user_data
            .as_ref()
            .is_some_and(|u| state::StateRepr::Running == u.state.get())
    }

    /// Returns the transfer's timeout. A value
    /// of `Duration::ZERO` indicates no timeout.
    pub const fn timeout(&self) -> Duration {
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

    fn cancel(&self) -> rusb::Result<()> {
        // SAFETY: `self.transfer` is a valid pointer
        //         for the entirety of self's lifetime.
        let result = unsafe { libusb_cancel_transfer(self.inner().as_raw()) };
        if 0 != result {
            Err(from_libusb(result))
        } else {
            Ok(())
        }
    }

    /// Returns `None` if the transfer type is not isochronous or if
    /// the transfer is not ready.
    pub fn iso_packets(&self) -> Option<&[impl IsoPacket]> {
        self.do_if_idle(|this| {
            if rusb::TransferType::Isochronous != this.kind() {
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
        })
        .flatten()
    }

    pub fn into_buf(mut self) -> Option<UsbMemMut> {
        self.do_if_idle_mut(|this| {
            this.user_data
                .take()
                .and_then(Arc::into_inner)
                .map(|u| u.buf.0.into_inner())
        })
        .flatten()
    }

    pub fn into_parts(mut self) -> Option<(InnerTransfer, UsbMemMut)> {
        self.do_if_idle_mut(|this| {
            let mut inner = this.inner.take().unwrap();
            // SAFETY: Inner transfer is no longer owned by `Transfer`.
            unsafe { inner.clear() };

            let buf = this
                .user_data
                .take()
                .and_then(Arc::into_inner)
                .map(|u| u.buf.0.into_inner())
                .unwrap();
            (inner, buf)
        })
    }
}

impl<C: rusb::UsbContext> Drop for Transfer<C> {
    fn drop(&mut self) {
        if self.is_running() {
            _ = self.cancel();
            // We take our these blocks of data so that
            // their destructors don't run when we drop
            // the outer transfer.
            let user_data = self.user_data.take().unwrap();
            let inner = self.inner.take().unwrap();
            match default_runtime() {
                Some(rt) => rt.spawn_detached(Box::pin(async move {
                    user_data.completion.notified().await;
                    let _goodbye = inner;
                })),
                None => {
                    // We leak the memory so that libusb nor
                    // the USB device will write into freed
                    // memory.
                    _ = Arc::into_raw(user_data);
                    std::mem::forget(inner);
                }
            };
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

fn default_runtime() -> Option<Arc<dyn AsyncRuntime>> {
    #[cfg(feature = "runtime-tokio")]
    {
        return ::tokio::runtime::Handle::try_current()
            .ok()
            .map(|handle| Arc::new(handle) as Arc<dyn AsyncRuntime>);
    }

    #[cfg(feature = "runtime-compio")]
    {
        return Some(Arc::new(CompioRuntime) as Arc<dyn AsyncRuntime>);
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

unsafe fn get_ctx(transfer: &libusb_transfer) -> &UserData {
    let user_data: *const UserData = transfer.user_data.cast_const().cast();
    // SAFETY: If we're not already in freed memory, then the user data
    // should be safe to access as well.
    unsafe { user_data.as_ref().unwrap_unchecked() }
}

/// Handles a USB transfer completion, cancellation, error, or whatever,
/// by simply alerting the function that submitted the transfer.
extern "system" fn transfer_callback(transfer: *mut libusb_transfer) {
    // SAFETY: `transfer` is a valid pointer and can be used to access
    //         `user_data`. `user_data` is a valid `Arc<UserData>` pointer
    //         created from `Arc::into_raw`, and can be turned back into
    //         a reference to the user data.
    let user_data = unsafe {
        let transfer = transfer.as_ref().unwrap_unchecked();
        get_ctx(transfer)
    };

    // notify_ready() locks the mutex while changing state
    user_data.notify_ready();
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
        _ => rusb::Error::Other,
    }
}

#[cfg(test)]
mod tests {
    use std::pin::pin;

    use crate::usb::DeviceHandle;

    use super::*;

    #[tokio::test]
    async fn submit_completes() {
        let handle: DeviceHandle<rusb::Context> = DeviceHandle::new();
        let buf = UsbMemMut::with_capacity(64);
        let inner_transfer = InnerTransfer::new(0);

        let transfer = unsafe { inner_transfer.into_int(&handle, 0, buf) };
        let cancel = CancellationToken::new();

        let result = tokio::select! {
            result = transfer.submit(&cancel) => {
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
        let inner_transfer = InnerTransfer::new(0);

        let transfer = unsafe { inner_transfer.into_int(&handle, 0, buf) };
        let cancel = CancellationToken::new();

        let mut fut = pin!(transfer.submit(&cancel));

        let result: Option<_> = tokio::select! {
            result = &mut fut => {
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
                let result = fut.await;
                assert_eq!(result, Ok(TransferStatus::Cancelled));
            }
        }
    }
}
