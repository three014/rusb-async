use std::{
    marker::PhantomData,
    os::fd::{BorrowedFd, OwnedFd},
    ptr::NonNull,
};

use compio_driver::{OpCode, OpEntry, Proactor};
use io_uring::{Submitter, opcode::PollAdd, types::Fd};
use rusb::{
    Context, UsbContext,
    ffi::{self, libusb_pollfd},
};

#[doc(hidden)]
macro_rules! try_unsafe {
    ($x:expr) => {
        match unsafe { $x } {
            0 => (),
            err => return Err($crate::from_libusb(err)),
        }
    };
}

// CONCEPT: Create an OpCode that returns one of these
// poll fds as an (Entry -> OpEntry), which can be polled
// repeatedly.

struct Runtime {
    libusb: NonNull<ffi::libusb_context>,
}

unsafe impl Send for Runtime {}

// How are we gonna handle our file descriptors?
// We first set our callback functions to handle new
// and removed descriptors. We then call a function to
// get all the current descriptors.
// I haven't decided whether to add all these into a list.

impl Runtime {
    #[inline]
    fn pollfds(&self) -> PollFds {
        let fds = unsafe { ffi::libusb_get_pollfds(self.libusb.as_ptr()) };
        PollFds {
            inner: NonNull::new(fds.cast_mut()).unwrap(),
        }
    }

    #[inline]
    fn new() -> rusb::Result<Self> {
        let mut ctx = std::mem::MaybeUninit::<*mut ffi::libusb_context>::uninit();
        try_unsafe!(ffi::libusb_init(ctx.as_mut_ptr()));
        Ok(unsafe { Self::from_raw(ctx.assume_init()) })
    }

    #[inline]
    unsafe fn from_raw(ctx: *mut ffi::libusb_context) -> Self {
        Self {
            libusb: unsafe { NonNull::new_unchecked(ctx) },
        }
    }
}

struct PollFds {
    inner: NonNull<*mut ffi::libusb_pollfd>,
}

impl PollFds {
    fn iter(&self) -> Iter<'_> {
        Iter {
            fd: unsafe { self.inner.as_ref() },
            _p: PhantomData,
        }
    }
}

struct Iter<'a> {
    fd: *const *mut ffi::libusb_pollfd,
    _p: PhantomData<&'a ffi::libusb_pollfd>,
}

impl<'a> Iterator for Iter<'a> {
    type Item = &'a ffi::libusb_pollfd;

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if (*self.fd).is_null() {
                None
            } else {
                let ptr = *self.fd;
                self.fd = self.fd.offset(1);
                Some(&*ptr)
            }
        }
    }
}

impl Drop for PollFds {
    fn drop(&mut self) {
        unsafe { ffi::libusb_free_pollfds(self.inner.as_ptr()) };
    }
}

struct LibusbPoll {
    fd: i32,
    events: i16,
}

impl From<&libusb_pollfd> for LibusbPoll {
    fn from(&libusb_pollfd { fd, events }: &libusb_pollfd) -> Self {
        Self { fd, events }
    }
}

impl From<libusb_pollfd> for LibusbPoll {
    fn from(libusb_pollfd { fd, events }: libusb_pollfd) -> Self {
        Self { fd, events }
    }
}

impl OpCode for LibusbPoll {
    fn create_entry(self: std::pin::Pin<&mut Self>) -> OpEntry {
        PollAdd::new(Fd(self.fd), self.events as _)
            .multi(true)
            .build()
            .into()
    }
}

fn foo() {
    let mut driver = Proactor::new().unwrap();
    let rt = Runtime::new().unwrap();

    for entry in rt.pollfds().iter().map(LibusbPoll::from) {
        let key = driver.push(entry);

        // GOAL: Define a paradigm where I can:
        // - Manage multiple devices with one "context/runtime"
        // - Have transfers be tied to the lifetime of a device
        //   (cleanup unfinished transfers)
        // - Not require an extra async/sync task to manage
        //   the libusb events
        //
        // Ideally, I'd like transfers to kind-of contain everything
        // they need to run, packing the device and context within
        // the transfer struct. Each transfer submits itself, then
        // waits until libusb wakes up the transfer future through the
        // callback function.
        // However, who waits for libusb? Who does compio wake up when
        // one of the pollfds returns with an event?
        //
        // 3/27/25: Transfers need an external task that polls for libusb
        // events, then calls the handle function to wake up the completed
        // futures. Polling externally prevents us from having to wake up
        // potentially N futures for every transfer that completes, aka
        // O(N^2) operation.
    }
}
