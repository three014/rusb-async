use std::{
    collections::VecDeque,
    hash::Hash,
    io,
    marker::PhantomData,
    ops::DerefMut,
    os::fd::RawFd,
    pin::Pin,
    ptr::NonNull,
    sync::{
        Arc, Mutex, MutexGuard, RwLock,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    task::{Poll, Waker, ready},
};

use atomic_waker::AtomicWaker;
use compio_buf::BufResult;
use compio_driver::{OpCode, OpEntry};
use compio_runtime::Submit;
use cooked_waker::{IntoWaker, WakeRef};
use futures_util::{
    FutureExt,
    future::{Fuse, FusedFuture},
};
use io_uring::{opcode::PollAdd, types::Fd};
use nohash_hasher::{IntMap, IntSet};
use pin_project_lite::pin_project;
use rusb::ffi::{self, libusb_pollfd};

type Id = usize;

trait PollFdNotifier {
    fn added(&self, pollfd: LibusbPoll);
    fn removed(&self, fd: RawFd);
}

// CONCEPT: Create an OpCode that returns one of these
// poll fds as an (Entry -> OpEntry), which can be polled
// repeatedly.

struct Runtime<N> {
    libusb: NonNull<ffi::libusb_context>,
    notifier: Arc<N>,
}

unsafe impl<N> Send for Runtime<N> {}

// How are we gonna handle our file descriptors?
// We first set our callback functions to handle new
// and removed descriptors. We then call a function to
// get all the current descriptors.
// I haven't decided whether to add all these into a list.

impl<N> Runtime<N> {
    #[inline]
    fn pollfds(&self) -> PollFds {
        let fds = unsafe { ffi::libusb_get_pollfds(self.libusb.as_ptr()) };
        PollFds {
            inner: Some(NonNull::new(fds.cast_mut()).unwrap()),
        }
    }

    /// Note that this function allocates space on the heap to store
    /// the notifier, then leaks the memory upon dropping the context.
    fn with_notifier(notifier: N) -> rusb::Result<Self>
    where
        N: PollFdNotifier,
    {
        use core::ffi::{c_int, c_short, c_void};

        let mut ctx = std::mem::MaybeUninit::<*mut ffi::libusb_context>::uninit();
        unsafe { crate::c_try(ffi::libusb_init(ctx.as_mut_ptr()))? };
        let libusb = unsafe { NonNull::new_unchecked(ctx.assume_init()) };

        // We increment the reference count when we clone here, which
        // will leak if we don't manually decrement the reference count
        // when we drop the whole runtime.
        let notifier = Arc::new(notifier);
        let user_data = Arc::into_raw(Arc::clone(&notifier));

        extern "system" fn added_cb<N: PollFdNotifier>(
            fd: c_int,
            events: c_short,
            user_data: *mut c_void,
        ) {
            let notifier = unsafe { &*user_data.cast::<N>() };
            notifier.added(LibusbPoll { fd, events });
        }

        extern "system" fn removed_cb<N: PollFdNotifier>(fd: c_int, user_data: *mut c_void) {
            let notifier = unsafe { &*user_data.cast::<N>() };
            notifier.removed(fd);
        }

        unsafe {
            ffi::libusb_set_pollfd_notifiers(
                libusb.as_ptr(),
                Some(added_cb::<N>),
                Some(removed_cb::<N>),
                user_data.cast_mut().cast::<c_void>(),
            )
        };

        Ok(Self { libusb, notifier })
    }

    fn handle_events(&self) -> rusb::Result<()> {
        unsafe {
            crate::c_try(ffi::libusb_handle_events_completed(
                self.libusb.as_ptr(),
                std::ptr::null_mut(),
            ))
        }
    }
}

impl<N> Drop for Runtime<N> {
    fn drop(&mut self) {
        // SAFETY: This just increments then decrements the reference count.
        // We do this to get a raw pointer to the notifier.
        let notifier = Arc::into_raw(Arc::clone(&self.notifier));
        unsafe { Arc::decrement_strong_count(notifier) };

        // SAFETY: We incremented the reference count to 2 when we cloned
        // the notifier for use with libusb, but can't retrieve the raw
        // pointer to convert back into an Arc unless we do what we did above.
        // Now, calling the decrement function will bring us back to a ref
        // count of 1, so when the Runtime's copy of the notifier drops, we
        // can drop the notifier for real.
        unsafe { Arc::decrement_strong_count(notifier) };
    }
}

struct PollFds {
    inner: Option<NonNull<*mut ffi::libusb_pollfd>>,
}

impl PollFds {
    fn iter(&self) -> Iter<'_> {
        Iter {
            fd: unsafe { self.inner.as_ref().unwrap().as_ref() },
            _p: PhantomData,
        }
    }
}

struct IntoIter {
    fd: *const *mut ffi::libusb_pollfd,
    _p: PhantomData<ffi::libusb_pollfd>,
}

impl Iterator for IntoIter {
    type Item = LibusbPoll;

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if (*self.fd).is_null() {
                return None;
            }

            let ptr = *self.fd;
            self.fd = self.fd.add(1);
            Some((&*ptr).into())
        }
    }
}

impl IntoIterator for PollFds {
    type Item = LibusbPoll;

    type IntoIter = IntoIter;

    fn into_iter(mut self) -> Self::IntoIter {
        IntoIter {
            fd: self.inner.take().unwrap().as_ptr().cast_const(),
            _p: PhantomData,
        }
    }
}

struct Iter<'a> {
    fd: *const *mut ffi::libusb_pollfd,
    _p: PhantomData<&'a ffi::libusb_pollfd>,
}

impl<'a> Iterator for Iter<'a> {
    type Item = LibusbPoll;

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if (*self.fd).is_null() {
                return None;
            }

            let ptr = *self.fd;
            self.fd = self.fd.add(1);
            Some((&*ptr).into())
        }
    }
}

impl Drop for PollFds {
    fn drop(&mut self) {
        if let Some(ptr) = self.inner.take() {
            unsafe { ffi::libusb_free_pollfds(ptr.as_ptr()) };
        }
    }
}

struct LibusbPoll {
    pub fd: RawFd,
    pub events: i16,
}

impl PartialEq for LibusbPoll {
    fn eq(&self, other: &Self) -> bool {
        self.fd == other.fd
    }
}

impl Hash for LibusbPoll {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.fd.hash(state);
    }
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

struct Entry<K, V> {
    key: K,
    value: V,
}

#[derive(Default)]
struct ActiveTransfers {
    inner: RwLock<Vec<Entry<Id, (AtomicWaker, AtomicBool)>>>,
}

impl ActiveTransfers {
    fn push_or_update(&self, id: usize, waker: &Waker) {
        if let Some((atomic, needs_to_handle_event)) = self
            .inner
            .read()
            .unwrap()
            .iter()
            .find(|entry| id == entry.key)
            .map(|entry| &entry.value)
        {
            atomic.register(waker);
            if needs_to_handle_event.load(Ordering::Relaxed) {
                atomic.wake();
            }
        } else {
            self.inner.write().unwrap().push(Entry {
                key: id,
                value: (AtomicWaker::new(), AtomicBool::new(false)),
            })
        }
    }

    fn handled_event(&self, id: usize) {
        if let Some((_, needs_to_handle_event)) = self
            .inner
            .read()
            .unwrap()
            .iter()
            .find(|entry| id == entry.key)
            .map(|entry| &entry.value)
        {
            needs_to_handle_event.store(false, Ordering::Relaxed);
        }
    }

    fn remove(&self, id: usize) {
        let mut wakers = self.inner.write().unwrap();
        if let Some(idx) = wakers.iter().position(|entry| id == entry.key) {
            let (_, needs_to_handle_event) = wakers.swap_remove(idx).value;
            if needs_to_handle_event.load(Ordering::Relaxed) {
                if let Some((next_waker, needs_to_handle_event)) =
                    wakers.first_mut().map(|entry| &mut entry.value)
                {
                    needs_to_handle_event.store(true, Ordering::Relaxed);
                    next_waker.wake();
                } else {
                    // Hmmmmmmmmm... TODO: What do we do when no one else can check on libusb?
                }
            }
        }
    }
}

impl cooked_waker::WakeRef for ActiveTransfers {
    fn wake_by_ref(&self) {
        if let Some((waker, is_woken)) =
            self.inner.read().unwrap().first().map(|entry| &entry.value)
        {
            is_woken.store(true, Ordering::Relaxed);
            waker.wake();
        }
    }
}

#[derive(Default)]
struct Notifier {
    retired: Arc<Mutex<Vec<RawFd>>>,
    new: Arc<Mutex<Entries>>,
}

impl PollFdNotifier for Notifier {
    fn added(&self, poll: LibusbPoll) {
        self.new.lock().unwrap().submit(poll);
    }

    fn removed(&self, fd: i32) {
        self.retired.lock().unwrap().push(fd);
    }
}

struct Context {
    active_entries: Arc<Mutex<Entries>>,
    retired_entries: Arc<Mutex<Vec<i32>>>,
    active_transfers: Arc<ActiveTransfers>,
    libusb: Runtime<Notifier>,
    curr_id: AtomicUsize,
}

impl Context {
    pub fn new() -> rusb::Result<Arc<Self>> {
        let retired_entries = Default::default();
        let active_entries = Default::default();
        let libusb = Runtime::with_notifier(Notifier {
            retired: Arc::clone(&retired_entries),
            new: Arc::clone(&active_entries),
        })?;

        {
            let mut entries = active_entries.lock().unwrap();
            if entries.active_is_empty() {
                entries.extend(libusb.pollfds().into_iter());
            }
        }

        Ok(Arc::new(Context {
            active_entries,
            retired_entries,
            active_transfers: Arc::new(ActiveTransfers {
                inner: RwLock::new(vec![]),
            }),
            libusb,
            curr_id: AtomicUsize::new(0),
        }))
    }

    pub fn next_id(&self) -> usize {
        self.curr_id.fetch_add(1, Ordering::Relaxed)
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        todo!()
    }
}

struct Submission {
    inner: Submit<LibusbPoll>,
}

impl Future for Submission {
    type Output = io::Result<LibusbPoll>;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        Pin::new(&mut self.get_mut().inner)
            .poll(cx)
            .map(|BufResult(result, poll)| match result {
                Ok(_) => Ok(poll),
                Err(err) => Err(err),
            })
    }
}

fn submit_entry(poll: LibusbPoll) -> Submission {
    Submission {
        inner: compio_runtime::submit(poll),
    }
}

#[derive(Default)]
struct Entries {
    active2: IntMap<RawFd, Submission>,
    ready2: Vec<(RawFd, io::Result<LibusbPoll>)>,
}

impl Entries {
    fn split_borrow_mut(
        &mut self,
    ) -> (
        &mut IntMap<RawFd, Submission>,
        &mut Vec<(RawFd, io::Result<LibusbPoll>)>,
    ) {
        (&mut self.active2, &mut self.ready2)
    }

    pub fn submit(&mut self, pollfd: LibusbPoll) {
        assert!(
            self.active2
                .insert(pollfd.fd, submit_entry(pollfd))
                .is_none()
        );
    }

    pub fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = LibusbPoll>,
    {
        self.active2.extend(
            iter.into_iter()
                .map(|pollfd| (pollfd.fd, submit_entry(pollfd))),
        );
    }

    fn handle_completed<F>(&mut self, mut predicate: F)
    where
        F: FnMut(io::Result<LibusbPoll>) -> Option<LibusbPoll>,
    {
        for pollfd in self
            .ready2
            .drain(..)
            .filter_map(|(_, result)| predicate(result))
        {
            assert!(
                self.active2
                    .insert(pollfd.fd, submit_entry(pollfd))
                    .is_none()
            );
        }
    }

    pub fn active_len(&self) -> usize {
        self.active2.len()
    }

    pub fn active_is_empty(&self) -> bool {
        self.active2.is_empty()
    }
}

impl Future for Entries {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let (active, ready) = self.split_borrow_mut();

        let completed = active
            .iter_mut()
            .filter_map(|(&fd, s)| match Pin::new(s).poll(cx) {
                Poll::Ready(e) => Some((fd, e)),
                Poll::Pending => None,
            });

        let start_idx = ready.len();
        ready.extend(completed);

        if let Some(ready) = ready.get(start_idx..) {
            for (fd, _) in ready {
                active.remove(&fd);
            }
        }

        if ready.is_empty() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

struct SelectAll<'a, F> {
    inner: &'a mut Vec<F>,
}

impl<F: Future + Unpin> Future for SelectAll<'_, F> {
    type Output = F::Output;

    fn poll(mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let item =
            self.inner
                .iter_mut()
                .enumerate()
                .find_map(|(i, f)| match Pin::new(f).poll(cx) {
                    Poll::Pending => None,
                    Poll::Ready(e) => Some((i, e)),
                });

        match item {
            Some((idx, res)) => {
                _ = self.inner.swap_remove(idx);
                Poll::Ready(res)
            }
            None => Poll::Pending,
        }
    }
}

pin_project! {
    struct Wait<'a> {
        #[pin]
        cancel: &'a mut crate::CancellationToken,
        cx: &'a Context,
        transfer: &'a crate::LibusbTransfer2,
        id: usize,
    }

    impl PinnedDrop for Wait<'_> {
        fn drop(this: Pin<&mut Self>) {
            this.cx.active_transfers.remove(this.id);
        }
    }
}

impl Future for Wait<'_> {
    type Output = crate::TransferStatus;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        use crate::state::StateRepr::*;
        let mut this = self.project();

        if Ready == this.transfer.footer().state().get() {
            // SAFETY: Transfer has completed, we are back
            // to being the only place that can read/write
            // to this data.
            return Poll::Ready(unsafe { this.transfer.status() });
        }

        // Every transfer future will do its part in driving
        // the libusb runtime by polling on libusb's file descriptors.

        this.transfer.footer().waker().register(cx.waker());
        this.cx
            .active_transfers
            .push_or_update(*this.id, cx.waker());
        loop {
            match this.transfer.footer().state().get() {
                Idle => {
                    if !this.transfer.footer().state().set_running() {
                        // SAFETY: Transfer has completed, we are back
                        // to being the only place that can read/write
                        // to this data.
                        return Poll::Ready(unsafe { this.transfer.status() });
                    }
                }
                Running => {
                    if !this.transfer.footer().is_cancelled()
                        && this.cancel.as_mut().poll(cx).is_ready()
                    {
                        _ = this.transfer.cancel();
                        this.transfer.footer().mark_cancelled();
                    }

                    // Every transfer must do their part in checking for
                    // events for libusb. With this we won't need an external
                    // runtime or thread managing the transfers.

                    let main_waker = this.cx.active_transfers.clone().into_waker();
                    let mut cx = std::task::Context::from_waker(&main_waker);

                    {
                        let mut entries = this.cx.active_entries.lock().unwrap();
                        ready!(Pin::new(entries.deref_mut()).poll(&mut cx))
                    }

                    // Rest of this block only matters if we were woken up to handle
                    // the libusb events. Otherwise, chances are that the next time we
                    // wake up, our transfer will be complete.

                    this.cx
                        .libusb
                        .handle_events()
                        .expect("this will also be an interesting error (headache)");
                    this.cx.active_transfers.handled_event(*this.id);

                    let mut active = this.cx.active_entries.lock().unwrap();
                    let mut retired = this.cx.retired_entries.lock().unwrap();

                    active.handle_completed(|result| {
                        let pollfd = result.expect("wonder how this happened?");

                        if let Some(idx) = retired.iter().position(|&fd| pollfd.fd == fd) {
                            retired.swap_remove(idx);
                            None
                        } else {
                            Some(pollfd)
                        }
                    });
                }
                // SAFETY: Transfer has completed, we are back
                // to being the only place that can read/write
                // to this data.
                Ready => unsafe {
                    return Poll::Ready(this.transfer.status());
                },
            }
        }
    }
}

struct Transfer3 {
    context: Arc<Context>,
    inner: crate::LibusbTransfer2,
}

impl Transfer3 {
    fn wait<'a>(&'a self, cancel: &'a mut crate::CancellationToken) -> Wait<'a> {
        Wait {
            cancel,
            id: self.context.next_id(),
            cx: &self.context,
            transfer: &self.inner,
        }
    }
}

fn _foo() {
    let retired_entries = Default::default();
    let active_entries = Default::default();
    let libusb = Runtime::with_notifier(Notifier {
        retired: Arc::clone(&retired_entries),
        new: Arc::clone(&active_entries),
    })
    .unwrap();
    let compio = compio_runtime::Runtime::new().unwrap();

    compio.block_on(async {
        // We get this list of entries to poll, but how should we
        // poll them? Ideally, this problem can be solved using a
        // trait I felt like I saw in the "futures" crate, where
        // we poll the list until one returns a `Ready`, then swap
        // out that future and replace it with a new one from the
        // same pollfd.
        {
            let mut entries = active_entries.lock().unwrap();
            if entries.active_is_empty() {
                entries.extend(libusb.pollfds());
            }
        }

        let context = Arc::new(Context {
            active_entries,
            retired_entries,
            active_transfers: Arc::new(ActiveTransfers {
                inner: RwLock::new(vec![]),
            }),
            libusb,
            curr_id: AtomicUsize::new(0),
        });

        // WRONG EXAMPLE. Correct example below this block.
        let _fut = std::future::poll_fn(|cx| {
            let mut entries = context.active_entries.lock().unwrap();

            // See here how we use the context given to us?
            // I don't really like that.
            //
            // I want to use our own context, which stores
            // a bag of wakers corresponding to unique transfer
            // IDs, so that when the waker is called, we can
            // dynamically select an active/valid transfer
            // future to wake up.
            //
            // What happens if our future isn't the one called
            // to handle the events? As long as we are a transfer
            // future, then our waker is also going to be stored
            // in the callback function of the transfer itself.
            // When another future handles the pending events, libusb
            // will then run all the applicable callback functions,
            // including this one, and by the time we poll ourselves,
            // our transfer will be complete and we don't need to
            // check here.
            Pin::new(entries.deref_mut()).poll(cx)
        });

        let id = context.curr_id.fetch_add(1, Ordering::Relaxed);
        let fut = std::future::poll_fn(|cx| {
            // We create an entry that IDs our transfer
            // and saves our waker in the bag of wakers.
            // This ID only needs to exist for the length of
            // the transfer future itself, not for the length
            // of the transfer as a whole.
            context.active_transfers.push_or_update(id, cx.waker());

            // We then create a context using the list of
            // active transfers.
            let main_waker = context.active_transfers.clone().into_waker();
            let mut cx = std::task::Context::from_waker(&main_waker);

            // Now we can poll all the entries for events.
            let mut entries = context.active_entries.lock().unwrap();
            Pin::new(entries.deref_mut()).poll(&mut cx)
        })
        .await;

        // From here, if our transfer isn't complete,
        // we get to call `libusb_handle_events`.
        context.libusb.handle_events().unwrap();
        context.active_transfers.handled_event(id);

        // Afterwards we resubmit the entry as long as
        // it's not listed in a "denylist" managed by
        // the libusb callback functions.

        {
            let mut active = context.active_entries.lock().unwrap();
            let mut retired = context.retired_entries.lock().unwrap();

            active.handle_completed(|result| {
                let pollfd = result.expect("wonder how this happened?");

                if let Some(idx) = retired.iter().position(|&fd| pollfd.fd == fd) {
                    retired.swap_remove(idx);
                    None
                } else {
                    Some(pollfd)
                }
            });
        }

        // Then we can check if our own transfer is complete.
        // If so then we can remove our waker and handle
        // our completed transfer, or wait again.
        //
        // (Pretend we're done and can remove the waker)
        context.active_transfers.remove(id);

        // There is another scenario we haven't mentioned, and it's
        // when a transfer future is dropped after having been woken
        // but before it handled the events. To address this we
        // need `Drop` code. TODO
        //
        // When dropping a transfer future, we need to remove ourselves
        // from the bag of wakers, but if we were called to handle the events,
        // then we pass on the responsibility to the next waker in the bag.
        // Surely this will work.

        // let future = compio_runtime::submit(entry);

        // What did we just do here?
        // I created a compio runtime and a libusb context, then
        // retrieved the poll descriptors from the context.
        // For each pollfd, I convert them into a compio OpCode,
        // then submit them to the runtime, which gives me back
        // a future to await. After doing this to each pollfd,
        // I'll now have a set of futures that I need to await
        // concurrently.
        //
        // The first question is "who awaits these futures?"
        //
        // Ideally, the first transfer submitted by the program
        // will be in charge of submitting these file descriptors,
        // and then the job of polling the returned futures will be shared
        // by all the subsequent transfers.
        //
        // Upon completion of this future,
        // I know that I am free to call the `handle_events` function
        // from libusb. I will continue to poll these descriptors until
        // libusb tells me to stop.
    });

    // driver.update_waker(op, waker);

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
    //
    // 4/1/25: Silly idea
    //
    // What if I made each transfer fight to be the one the async
    // executor wakes up when a poll returns an event??
    //
    // 4/21/25: Continuing on this idea:
    //
    // waiters: A simple list (Vec) of Wakers.
    // - how will it be used?
    //   - Futures will add their own waker to the list
    //   - On wake, the first future to obtain a permit will
    //     call libusb_handle_events and check if their transfer
    //     is complete
    //   - If not complete, reenter the queue and await a wakeup
    //   - If complete, remove self from the queue and return result
    //
}
