use std::{marker::PhantomData, ops::Deref, ptr::NonNull};

use rusb::ffi::{libusb_dev_mem_alloc, libusb_dev_mem_free, libusb_device_handle};

#[repr(transparent)]
pub struct BytePtr {
    inner: NonNull<u8>,
    _p: PhantomData<u8>,
}

unsafe impl Send for BytePtr {}
unsafe impl Sync for BytePtr {}

pub struct DevMem {
    ptr: BytePtr,
    len: usize,
    dev_handle: Option<NonNull<libusb_device_handle>>,
}

unsafe impl Send for DevMem {}
unsafe impl Sync for DevMem {}

impl Deref for DevMem {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.ptr.inner.as_ptr(), self.len) }
    }
}

impl Drop for DevMem {
    fn drop(&mut self) {
        let dev_handle = match self.dev_handle {
            Some(ptr) => ptr.as_ptr(),
            None => unreachable!(),
        };

        unsafe {
            libusb_dev_mem_free(dev_handle, self.ptr.inner.as_ptr(), self.len);
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AllocError {
    NotSupported,
}

pub trait DeviceHandleExt {
    /// GOAL: Return something that can give me nonoverlapping `&mut [MaybeUninit<u8>]`s,
    ///       just like BytesMut, but that allows mapped memory as well (Bytes doesn't
    ///       allow conversion to BytesMut from static/mapped memory).
    unsafe fn alloc_dev_mem(&self, len: usize) -> Result<DevMem, AllocError>;
}

impl<T: rusb::UsbContext> DeviceHandleExt for rusb::DeviceHandle<T> {
    unsafe fn alloc_dev_mem(&self, len: usize) -> Result<DevMem, AllocError> {
        let dev_handle = self.as_raw();
        let ptr = unsafe { libusb_dev_mem_alloc(dev_handle, len) };
        match NonNull::new(ptr) {
            Some(inner) => {
                let mem = DevMem {
                    ptr: BytePtr {
                        inner,
                        _p: PhantomData,
                    },
                    len,
                    dev_handle: Some(NonNull::new(dev_handle).unwrap()),
                };

                Ok(mem)
            }
            None => Err(AllocError::NotSupported),
        }
    }
}
