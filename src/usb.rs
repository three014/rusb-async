#[cfg(all(not(test), feature = "dma"))]
pub use crate::dma::UsbMemMut;
#[cfg(all(not(test), not(feature = "dma")))]
pub use bytes::BytesMut as UsbMemMut;

#[cfg(not(test))]
pub use rusb::{
    DeviceHandle,
    ffi::{
        libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer, libusb_submit_transfer,
    },
};
#[cfg(test)]
pub use tb::{
    DeviceHandle, UsbMemMut, libusb_alloc_transfer, libusb_cancel_transfer, libusb_free_transfer,
    libusb_submit_transfer,
};

#[cfg(test)]
mod tb {
    use std::{ffi::c_int, marker::PhantomData, time::Duration};

    use rusb::ffi;

    use crate::Footer;

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

    const TEST_NUM_PACKETS: usize = 6 + crate::FOOTER_SIZE_IN_PACKETS;
    const RAW_SIZE: usize = size_of::<ffi::libusb_transfer>()
        + TEST_NUM_PACKETS * size_of::<ffi::libusb_iso_packet_descriptor>();

    #[repr(C, align(8))]
    struct Test {
        inner: [u8; RAW_SIZE],
    }

    impl Test {
        const fn new() -> Self {
            Self {
                inner: [0; RAW_SIZE],
            }
        }
    }

    pub unsafe fn libusb_alloc_transfer(_iso_packets: c_int) -> *mut ffi::libusb_transfer {
        assert!(
            TEST_NUM_PACKETS >= _iso_packets as usize,
            "Can't allocate larger structs than this for testing, sorry"
        );
        Box::into_raw(Box::new(Test::new())).cast()
    }

    pub unsafe fn libusb_free_transfer(transfer: *mut ffi::libusb_transfer) {
        _ = unsafe { Box::from_raw(transfer.cast::<Test>()) }
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
        let footer = Footer::ref_from_transfer(t);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(745)).await;
            footer.state().set_ready();
            footer.waker().wake();
        });
        0
    }
}
