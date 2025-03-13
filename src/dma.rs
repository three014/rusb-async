use core::slice;
use std::{borrow::{Borrow, BorrowMut}, cmp, fmt::Debug, mem::MaybeUninit, ops::{Deref, DerefMut}, ptr::NonNull, sync::Arc};

use bytes::buf::IntoIter;
use ::bytes::{Buf, BufMut};
use rusb::ffi::{libusb_dev_mem_alloc, libusb_dev_mem_free, libusb_device_handle};

use crate::Ptr;

pub struct UsbMemMut {
    ptr: NonNull<u8>,
    len: u32,
    cap: u32,
    data: Arc<RawUsbMem>,
}

impl std::fmt::Debug for UsbMemMut {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.as_ref().fmt(f)
    }
}

impl UsbMemMut {
    #[inline]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    pub const fn capacity(&self) -> usize {
        self.cap as usize
    }

    #[inline]
    const fn data(&self) -> &Arc<RawUsbMem> {
        &self.data
    }

    #[inline]
    pub const fn clear(&mut self) {
        unsafe { self.set_len(0) };
    }

    #[inline]
    pub const fn truncate(&mut self, len: usize) {
        if len <= self.len() {
            unsafe { self.set_len(len) };
        }
    }

    #[inline]
    pub const unsafe fn set_len(&mut self, len: usize) {
        debug_assert!(len <= self.capacity(), "set_len out of bounds");
        self.len = len as u32;
    }

    #[must_use = "consider UsbMemMut::truncate if you don't need the other half"]
    pub fn split_off(&mut self, at: usize) -> Self {
        assert!(
            at <= self.capacity(),
            "split_off out of bounds: {:?} <= {:?}",
            at,
            self.capacity()
        );
        unsafe {
            let mut other = self.shallow_clone();
            other.advance_unchecked(at);
            self.cap = at as u32;
            self.len = cmp::min(self.len, at as u32);
            other
        }
    }

    #[inline]
    pub fn split(&mut self) -> Self {
        let len = self.len();
        self.split_to(len)
    }

    pub fn split_to(&mut self, to: usize) -> Self {
        assert!(
            to <= self.len(),
            "split_to out of bounds: {:?} <= {:?}",
            to,
            self.len(),
        );

        unsafe {
            let mut other = self.shallow_clone();
            self.advance_unchecked(to);
            other.cap = to as u32;
            other.len = to as u32;
            other
        }
    }

    unsafe fn advance_unchecked(&mut self, cnt: usize) {
        if cnt == 0 {
            return;
        }

        debug_assert!(cnt <= self.capacity(), "internal: set_start out of bounds");

        self.ptr = vptr(unsafe { self.ptr.as_ptr().add(cnt) });
        self.len = self.len.saturating_sub(cnt as u32);
        self.cap -= cnt as u32;
    }

    unsafe fn shallow_clone(&self) -> Self {
        Self {
            ptr: self.ptr,
            len: self.len,
            cap: self.cap,
            data: Arc::clone(self.data())
        }
    }

    #[inline]
    fn as_slice(&self) -> &[u8] {
        unsafe { slice::from_raw_parts(self.ptr.as_ptr(), self.len()) }
    }

    #[inline]
    fn as_slice_mut(&mut self) -> &mut [u8] {
        unsafe { slice::from_raw_parts_mut(self.ptr.as_ptr(), self.len()) }
    }

    pub fn try_extend_from_slice(&mut self, extend: &[u8]) -> bool {
        let cnt = extend.len();
        if !self.try_reclaim(cnt) {
            return false;
        }

        unsafe {
            let dst = self.spare_capacity_mut();
            debug_assert!(dst.len() >= cnt);

            core::ptr::copy_nonoverlapping(extend.as_ptr(), dst.as_mut_ptr().cast(), cnt);
        }

        unsafe {
            self.advance_mut(cnt);
        }

        true
    }

    #[inline]
    pub fn spare_capacity_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        unsafe {
            let ptr = self.ptr.as_ptr().add(self.len());
            let len = self.capacity() - self.len();

            slice::from_raw_parts_mut(ptr.cast(), len)
        }
    }

    #[inline]
    fn offset(&self) -> usize {
        let off = unsafe { self.ptr.as_ptr().offset_from(self.data().ptr.as_ptr())};
        assert!(!off.is_negative());
        off as usize
    }

    fn reserve_inner(&mut self, additional: usize) -> bool {
        let off = self.offset();

        match Arc::strong_count(self.data()) {
            1 if self.capacity() - self.len() + off >= additional && off >= self.len() => unsafe {
                let base_ptr = self.ptr.as_ptr().sub(off);
                core::ptr::copy_nonoverlapping(self.ptr.as_ptr(), base_ptr, self.len());
                self.ptr = vptr(base_ptr);
                self.cap += off as u32;
                true
            },
            _ => false,
        }
    }

    #[inline]
    pub fn try_reclaim(&mut self, additional: usize) -> bool {
        let len = self.len();
        let rem = self.capacity() - len;

        if additional <= rem {
            return true;
        }

        self.reserve_inner(additional)
    }
}

impl Buf for UsbMemMut {
    #[inline]
    fn remaining(&self) -> usize {
        self.len()
    }

    #[inline]
    fn chunk(&self) -> &[u8] {
        self.as_slice()
    }

    #[inline]
    fn advance(&mut self, cnt: usize) {
        assert!(
            cnt <= self.remaining(),
            "cannot advance past `remaining`: {:?} <= {:?}",
            cnt,
            self.remaining()
        );
        unsafe {
            self.advance_unchecked(cnt);
        }
    }
}

unsafe impl BufMut for UsbMemMut {
    #[inline]
    fn remaining_mut(&self) -> usize {
        self.capacity() - self.len()
    }

    #[inline]
    unsafe fn advance_mut(&mut self, cnt: usize) {
        let remaining = self.capacity() - self.len();
        if cnt > remaining {
            panic!(
                "advance out of bounds: the len is {} but advancing by {}",
                remaining, cnt
            );
        }
        self.len += cnt as u32;
    }

    #[inline]
    fn chunk_mut(&mut self) -> &mut ::bytes::buf::UninitSlice {
        self.spare_capacity_mut().into()
    }

    fn put<T: Buf>(&mut self, mut src: T)
    where
        Self: Sized,
    {
        while src.has_remaining() {
            let s = src.chunk();
            let l = s.len();
            assert!(self.try_extend_from_slice(s));
            src.advance(l);
        }
    }

    fn put_slice(&mut self, src: &[u8]) {
        assert!(self.try_extend_from_slice(src));
    }

    fn put_bytes(&mut self, val: u8, cnt: usize) {
        assert!(self.try_reclaim(cnt));
        unsafe {
            let dst = self.spare_capacity_mut();
            debug_assert!(dst.len() >= cnt);

            core::ptr::write_bytes(dst.as_mut_ptr(), val, cnt);

            self.advance_mut(cnt);
        }
    }
}

impl AsRef<[u8]> for UsbMemMut {
    #[inline]
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl Deref for UsbMemMut {
    type Target = [u8];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl AsMut<[u8]> for UsbMemMut {
    #[inline]
    fn as_mut(&mut self) -> &mut [u8] {
        self.as_slice_mut()
    }
}

impl DerefMut for UsbMemMut {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut()
    }
}

impl Borrow<[u8]> for UsbMemMut {
    fn borrow(&self) -> &[u8] {
        self.as_ref()
    }
}

impl BorrowMut<[u8]> for UsbMemMut {
    fn borrow_mut(&mut self) -> &mut [u8] {
        self.as_mut()
    }
}

impl IntoIterator for UsbMemMut {
    type Item = u8;
    type IntoIter = IntoIter<UsbMemMut>;

    fn into_iter(self) -> Self::IntoIter {
        IntoIter::new(self)
    }
}

impl<'a> IntoIterator for &'a UsbMemMut {
    type Item = &'a u8;
    type IntoIter = core::slice::Iter<'a, u8>;

    fn into_iter(self) -> Self::IntoIter {
        self.as_ref().iter()
    }
}

unsafe impl Send for UsbMemMut {}
unsafe impl Sync for UsbMemMut {}

#[inline]
fn vptr(ptr: *mut u8) -> NonNull<u8> {
    unsafe { NonNull::new_unchecked(ptr) }
}

struct RawUsbMem {
    ptr: Ptr<u8>,
    len: usize,
    dev_handle: NonNull<libusb_device_handle>,
}

impl Deref for RawUsbMem {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr().cast_const(), self.len) }
    }
}

impl Drop for RawUsbMem {
    fn drop(&mut self) {
        let dev_handle = self.dev_handle.as_ptr();

        unsafe {
            libusb_dev_mem_free(dev_handle, self.ptr.as_ptr(), self.len);
        }
    }
}

#[derive(Debug, Clone, Copy)]
/// A struct representing an allocation error from libusb.
pub struct AllocError;

pub trait DeviceHandleExt {
    /// # Safety
    ///
    /// It is undefined behaviour to destroy the associated device handle
    /// before freeing this block of memory.
    ///
    /// # Panic
    ///
    /// This function panics on debug-mode if `len` is larger than `u32::MAX`.
    unsafe fn new_usb_mem(&self, len: usize) -> Result<UsbMemMut, AllocError>;
}

impl<T: rusb::UsbContext> DeviceHandleExt for rusb::DeviceHandle<T> {
    unsafe fn new_usb_mem(&self, len: usize) -> Result<UsbMemMut, AllocError> {
        debug_assert!(len < u32::MAX as usize);
        let dev_handle = self.as_raw();
        let ptr = unsafe { libusb_dev_mem_alloc(dev_handle, len) };
        match Ptr::new(ptr) {
            Some(ptr) => {
                let raw_mem = RawUsbMem {
                    ptr,
                    len,
                    // SAFETY: Since the ptr is non-null, that means the
                    // allocation succeeded and we're free to use this pointer.
                    dev_handle: unsafe { NonNull::new_unchecked(dev_handle) },
                };
                Ok(UsbMemMut {
                    ptr: ptr.as_non_null_ptr(),
                    len: 0,
                    cap: len as u32,
                    data: Arc::new(raw_mem),
                })
            }
            None => Err(AllocError),
        }
    }
}
