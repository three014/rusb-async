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
    libusb_alloc_transfer, libusb_fill_control_transfer, libusb_fill_interrupt_transfer,
    libusb_free_transfer, libusb_submit_transfer, libusb_transfer,
};
use std::time::Duration;
use std::{ffi::c_void, marker::PhantomData, ptr::NonNull};
use tokio::sync::oneshot;
use tokio_util::sync::CancellationToken;

use zerocopy_derive::*;

mod dma;

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

#[derive(Debug)]
pub struct Transfer<C: rusb::UsbContext> {
    inner: Option<NonNull<libusb_transfer>>,
    notifier: oneshot::Receiver<NonNull<libusb_transfer>>,
    buf: BytesMut,
    _ctx: PhantomData<C>,
}

impl<C: rusb::UsbContext> Transfer<C> {
    pub(crate) fn alloc_raw(num_isos: usize) -> NonNull<libusb_transfer> {
        assert!(u16::MAX as usize > num_isos);

        // SAFETY: libusb allocates the data, and returns a null ptr
        //         if the allocation somehow failed.
        let ptr = unsafe { libusb_alloc_transfer(num_isos as i32) };

        NonNull::new(ptr).unwrap()
    }

    /// Allocates a [`libusb_transfer`] using FFI, then populates
    /// the fields with the supplied handle and buffer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device.
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_interrupt_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    pub unsafe fn fill_int(
        dev_handle: &rusb::DeviceHandle<C>,
        endpoint: u8,
        mut buf: BytesMut,
    ) -> Self {
        debug_assert_eq!(
            size_of::<Box<oneshot::Sender<NonNull<libusb_transfer>>>>(),
            size_of::<*mut c_void>()
        );
        let transfer = Self::alloc_raw(0);
        let (tx, rx) = oneshot::channel();
        let user_data = Box::into_raw(Box::new(tx));

        // SAFETY: Caller ensures that the capacity of `buf`
        //         is exactly the size of the buffer they want
        //         passed to the USB device.
        unsafe { buf.set_len(0) };

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_interrupt_transfer(
                transfer.as_ptr(),
                dev_handle.as_raw(),
                endpoint,
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                buf.spare_capacity_mut().len() as i32,
                transfer_callback,
                user_data.cast(),
                u32::MAX,
            )
        };

        Self {
            inner: Some(transfer),
            notifier: rx,
            buf,
            _ctx: PhantomData,
        }
    }

    /// Allocates a [`libusb_transfer`] using FFI, then populates
    /// the fields with the supplied data.
    ///
    /// `timeout` will be truncated to `Duration::from_millis(u32::MAX)`
    /// if greater than `u32::MAX` milliseconds. A timeout of `Duration::ZERO`
    /// indicates no timeout.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the capacity of `buf` is exactly
    /// the size of the buffer they want passed to the USB device.
    ///
    /// ALSO, the caller must make sure that the first 8 bytes of `buf`
    /// contain the [`ControlPacket`], and that:
    /// ```
    /// ControlPacket::w_length + size_of::<ControlPacket>() == buf.capacity()
    /// ```
    ///
    /// Regardless if `buf` contains initialized data, its length will be set to 0
    /// so that the entire slice can be passed to `libusb_fill_interrupt_transfer`
    /// as a `*mut MaybeUninit<u8>`.
    pub unsafe fn fill_ctrl(
        dev_handle: &rusb::DeviceHandle<C>,
        mut buf: BytesMut,
        timeout: Duration,
    ) -> Self {
        debug_assert_eq!(
            size_of::<Box<oneshot::Sender<NonNull<libusb_transfer>>>>(),
            size_of::<*mut c_void>()
        );
        let transfer = Self::alloc_raw(0);
        let (tx, rx) = oneshot::channel();
        let user_data = Box::into_raw(Box::new(tx));
        let timeout = timeout.clamp(Duration::ZERO, Duration::from_millis(u32::MAX as u64));

        // SAFETY: Caller ensures that the capacity of `buf`
        //         is exactly the size of the buffer they want
        //         passed to the USB device.
        unsafe { buf.set_len(0) };

        // SAFETY: All data is valid for access by libusb.
        unsafe {
            libusb_fill_control_transfer(
                transfer.as_ptr(),
                dev_handle.as_raw(),
                buf.spare_capacity_mut().as_mut_ptr().cast(),
                transfer_callback,
                user_data.cast(),
                timeout.as_millis() as u32,
            )
        };

        Self {
            inner: Some(transfer),
            notifier: rx,
            buf,
            _ctx: PhantomData,
        }
    }

    pub async fn submit(
        &mut self,
        cancel_token: CancellationToken,
    ) -> Result<TransferStatus, rusb::Error> {
        // SAFETY: We hold exclusive mutable access to `self`.
        //         Therefore, we can prevent anyone else from
        //         accessing the BytesMut and the inner transfer
        //         while libusb uses the data.
        let result = unsafe { libusb_submit_transfer(self.inner.take().unwrap().as_ptr()) };
        if 0 != result {
            return Err(from_libusb(result));
        }

        enum Event {
            Cancelled,
            Completed(NonNull<libusb_transfer>),
        }

        let transfer = match tokio::select! {
            biased;
            _ = cancel_token.cancelled() => Event::Cancelled,
            transfer = &mut self.notifier => Event::Completed(transfer.unwrap()),
        } {
            Event::Cancelled => (&mut self.notifier).await.unwrap(),
            Event::Completed(transfer) => transfer,
        };

        // SAFETY: libusb has completed the transfer and is no
        //         longer using the transfer struct, making it
        //         safe for us to use.
        //         Furthermore, transfer is a well-aligned and valid
        //         pointer, making it safe to access.
        unsafe {
            // Update BytesMut with the new buffer length from transfer
            let transfer_ref = transfer.as_ref();
            let mut transfer_len = transfer_ref.actual_length as usize;
            if LIBUSB_TRANSFER_TYPE_CONTROL == transfer_ref.transfer_type {
                transfer_len += size_of::<ControlPacket>();
            }
            self.buf.set_len(self.buf.len() + transfer_len);

            // Get status from transfer
            let status = TransferStatus::from_i32(transfer_ref.status).unwrap();

            // Put transfer back into self
            self.inner = Some(transfer);

            Ok(status)
        }
    }

    /// Returns the transfer's timeout. A value
    /// of `Duration::ZERO` indicates no timeout.
    pub const fn timeout(&self) -> Duration {
        // SAFETY: `self.inner` is a valid and initialized
        //         pointer to a `libusb_transfer`
        unsafe {
            match self.inner {
                Some(inner) => Duration::from_millis(inner.as_ref().timeout as u64),
                None => unreachable!(),
            }
        }
    }

    /// Returns the transfer's `rusb::TransferType`.
    pub const fn kind(&self) -> rusb::TransferType {
        unsafe {
            match self.inner {
                Some(inner) => match inner.as_ref().transfer_type {
                    LIBUSB_TRANSFER_TYPE_CONTROL => rusb::TransferType::Control,
                    LIBUSB_TRANSFER_TYPE_INTERRUPT => rusb::TransferType::Interrupt,
                    LIBUSB_TRANSFER_TYPE_BULK => rusb::TransferType::Bulk,
                    LIBUSB_TRANSFER_TYPE_ISOCHRONOUS => rusb::TransferType::Isochronous,
                    _ => unreachable!(),
                },
                None => unreachable!(),
            }
        }
    }

    /// Returns a slice containing the transfer's initialized buffer.
    /// During a call to [`Transfer::submit`], this function returns
    /// an empty slice.
    pub fn buffer(&self) -> &[u8] {
        match self.inner {
            Some(_) => &self.buf,
            None => &[]
        }
    }
}

impl<T: rusb::UsbContext> Drop for Transfer<T> {
    fn drop(&mut self) {
        match self.inner {
            // SAFETY: libusb allocated this buffer,
            //         so they get to free it.
            Some(ptr) => unsafe {
                libusb_free_transfer(ptr.as_ptr());
            },
            // It's unlikely, but still possible for now that
            // `Transfer::submit` might panic, so we do nothing
            // here to prevent double panic.
            None => (),
        }
    }
}

extern "system" fn transfer_callback(transfer: *mut libusb_transfer) {
    // SAFETY: `transfer` is a valid pointer and can be used to access
    //         `user_data`. `user_data` is a valid
    //         `Box<oneshot::Sender<NonNull<libusb_transfer>>>` pointer
    //         created from `Box::into_raw`, and can be turned back into
    //         a box.
    let notifier = unsafe {
        let user_data: *mut oneshot::Sender<NonNull<libusb_transfer>> =
            (*transfer).user_data.cast();
        Box::from_raw(user_data)
    };
    _ = notifier.send(NonNull::new(transfer).unwrap());
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
        LIBUSB_ERROR_OTHER | _ => rusb::Error::Other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {}
}
