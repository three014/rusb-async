#[cfg(not(test))]
pub use crate::dma::UsbMemMut;
#[cfg(not(test))]
pub use rusb::{
    ffi::{
        libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer, libusb_submit_transfer,
    },
    DeviceHandle,
};
#[cfg(test)]
pub use tb::{
    libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer, libusb_submit_transfer,
    DeviceHandle, UsbMemMut,
};

#[cfg(test)]
mod tb {
    use std::{ffi::c_int, marker::PhantomData, sync::Arc, time::Duration};

    use rusb::ffi;

    use crate::{dummy_callback, LibusbState, UserData};

    pub use bytes::BytesMut as UsbMemMut;

    #[derive(Debug, Default)]
    pub struct DeviceHandle<T: rusb::UsbContext> {
        _p: PhantomData<T>,
    }

    impl<T: rusb::UsbContext> DeviceHandle<T> {
        pub fn as_raw(&self) -> *mut ffi::libusb_device_handle {
            std::ptr::NonNull::dangling().as_ptr()
        }

        pub fn new() -> Self {
            Self { _p: PhantomData }
        }
    }

    pub unsafe fn libusb_alloc_transfer(_iso_packets: c_int) -> *mut ffi::libusb_transfer {
        assert_eq!(_iso_packets, 0, "Can't allocate dynamic slices like that for testing, sorry");
        Box::into_raw(Box::new(ffi::libusb_transfer {
            dev_handle: std::ptr::null_mut(),
            flags: 0,
            endpoint: 0,
            transfer_type: 0,
            timeout: 0,
            status: 0,
            length: 0,
            actual_length: 0,
            callback: dummy_callback,
            user_data: std::ptr::null_mut(),
            buffer: std::ptr::null_mut(),
            num_iso_packets: 0,
            iso_packet_desc: [],
        }))
    }

    pub unsafe fn libusb_free_transfer(transfer: *mut ffi::libusb_transfer) {
        _ = unsafe { Box::from_raw(transfer) }
    }

    pub unsafe fn libusb_cancel_transfer(transfer: *mut ffi::libusb_transfer) -> c_int {
        let t = unsafe { transfer.as_mut().unwrap() };
        t.status = rusb::constants::LIBUSB_TRANSFER_CANCELLED;
        t.actual_length = 0;
        0
    }

    pub unsafe fn libusb_submit_transfer(transfer: *mut ffi::libusb_transfer) -> c_int {
        let t = unsafe { transfer.as_mut().unwrap() };
        t.status = rusb::constants::LIBUSB_TRANSFER_COMPLETED;
        t.actual_length = t.length;
        let ptr: *const UserData = t.user_data.cast_const().cast();
        let transfer_user_data = unsafe { Arc::from_raw(ptr) };
        let task_user_data = Arc::clone(&transfer_user_data);
        _ = Arc::into_raw(transfer_user_data);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(745)).await;
            let mut state = task_user_data.state.lock().unwrap();
            task_user_data.completion.notify_one();
            *state = LibusbState::Ready;
            drop(state);
        });
        0
    }
}
