use std::{
    io,
    marker::PhantomData,
    mem::ManuallyDrop,
    os::fd::RawFd,
    pin::Pin,
    ptr::NonNull,
    sync::{
        Arc, Mutex, RwLock,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    task::{Poll, Waker, ready},
};

use atomic_waker::AtomicWaker;
use compio_buf::BufResult;
use compio_driver::{OpCode, OpEntry};
use compio_runtime::Submit;
use cooked_waker::IntoWaker;
use io_uring::{opcode::PollAdd, types::Fd};
use pin_project_lite::pin_project;
use rusb::ffi::{self, libusb_pollfd};

use crate::state::StateRepr;

type Id = usize;

trait PollFdNotifier {
    fn added(&self, pollfd: LibusbPoll);
    fn removed(&self, fd: RawFd);
}

// CONCEPT: Create an OpCode that returns one of these
// poll fds as an (Entry -> OpEntry), which can be polled
// repeatedly.

struct Context<N> {
    libusb: NonNull<ffi::libusb_context>,
    notifier: Arc<N>,
}

unsafe impl<N> Send for Context<N> {}

// How are we gonna handle our file descriptors?
// We first set our callback functions to handle new
// and removed descriptors. We then call a function to
// get all the current descriptors.
// I haven't decided whether to add all these into a list.

impl<N> Context<N> {
    #[inline]
    fn pollfds(&self) -> PollFds {
        let fds = unsafe { ffi::libusb_get_pollfds(self.libusb.as_ptr()) };
        PollFds {
            inner: Some(NonNull::new(fds.cast_mut()).unwrap()),
        }
    }

    /// Initializes a new libusb context and installs the
    /// provided notifier to handle new and removed poll
    /// file descriptors.
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

    /// Calls into `libusb_handle_events_completed` with
    /// no timeout, returning immediately if there are no
    /// events to handle.
    fn handle_events(&self) -> rusb::Result<()> {
        unsafe {
            crate::c_try(ffi::libusb_handle_events_completed(
                self.libusb.as_ptr(),
                std::ptr::null_mut(),
            ))
        }
    }
}

impl<N> Drop for Context<N> {
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

struct IntoIter {
    start: *const *mut ffi::libusb_pollfd,
    idx: usize,
    _p: PhantomData<ffi::libusb_pollfd>,
}

impl Iterator for IntoIter {
    type Item = LibusbPoll;

    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            let next = (*self.start).add(self.idx);
            self.idx += 1;
            if next.is_null() {
                None
            } else {
                Some((&*next).into())
            }
        }
    }
}

impl Drop for IntoIter {
    fn drop(&mut self) {
        unsafe { ffi::libusb_free_pollfds(self.start) };
    }
}

impl IntoIterator for PollFds {
    type Item = LibusbPoll;

    type IntoIter = IntoIter;

    fn into_iter(mut self) -> Self::IntoIter {
        IntoIter {
            start: self.inner.take().unwrap().as_ptr().cast_const(),
            idx: 0,
            _p: PhantomData,
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
            let atomic = AtomicWaker::new();
            atomic.register(waker);
            self.inner.write().unwrap().push(Entry {
                key: id,
                value: (atomic, AtomicBool::new(false)),
            })
        }
    }

    /// Marks all outstanding libusb events as "handled"
    /// so that other futures don't unnecessarily call
    /// into libusb.
    fn handled_events(&self) {
        self.inner
            .read()
            .unwrap()
            .iter()
            .map(|entry| &entry.value)
            .for_each(|(_, needs_to_handle_event)| {
                needs_to_handle_event.store(false, Ordering::Relaxed)
            });
    }

    /// Removes the waker associated with the `id` from the
    /// list of futures that can be called upon to handle
    /// new libusb events. If the removed waker was called
    /// upon to handle events, then the function wakes up
    /// the next available waker.
    fn remove(&self, id: usize) {
        let mut wakers = self.inner.write().unwrap();
        if let Some((_, needs_to_handle_event)) = wakers
            .iter()
            .position(|entry| id == entry.key)
            .map(|idx| wakers.swap_remove(idx).value)
        {
            if needs_to_handle_event.load(Ordering::Relaxed) {
                if let Some((next_waker, needs_to_handle_event)) =
                    self.inner.read().unwrap().first().map(|entry| &entry.value)
                {
                    needs_to_handle_event.store(true, Ordering::Relaxed);
                    next_waker.wake();
                } else {
                    // TODO: What to do when no one else can check on libusb??
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
    entries: Arc<Mutex<AllEntries>>,
}

impl PollFdNotifier for Notifier {
    fn added(&self, poll: LibusbPoll) {
        self.entries.lock().unwrap().active_and_ready.submit(poll);
    }

    fn removed(&self, fd: RawFd) {
        self.entries.lock().unwrap().retired.push(fd);
    }
}

/// Forgotten/dropped transfers that haven't
/// completed at the drop time.
///
/// The buffer's held by these transfers might
/// get written to by USB devices or libusb,
/// and should not be deallocated, or else
/// UAE occurs. The owner of this struct
/// should ensure all transfers are complete
/// before dropping the inner vec.
#[derive(Default)]
struct RetiredTransfers {
    inner: Mutex<Vec<crate::LibusbTransfer2>>,
}

#[derive(Default)]
struct AllEntries {
    active_and_ready: Entries,
    retired: Vec<RawFd>,
}

impl AllEntries {
    fn split_borrow_mut(&mut self) -> (&mut Entries, &mut Vec<RawFd>) {
        (&mut self.active_and_ready, &mut self.retired)
    }

    fn update(&mut self) {
        let (active_and_ready, retired) = self.split_borrow_mut();
        let (active, ready) = active_and_ready.split_borrow_mut();

        let ready = ready
            .drain(..)
            .filter_map(|(_, result)| match result {
                Ok(poll) => {
                    if let Some(idx) = retired.iter().position(|&fd| poll.fd == fd) {
                        retired.swap_remove(idx);
                        None
                    } else {
                        Some(poll)
                    }
                }
                Err(err) => unimplemented!("{err}"),
            })
            .map(submit_entry);

        active.extend(ready);
    }
}

struct _WaitForAll<'a> {
    active_transfers: &'a ActiveTransfers,
    retired_transfers: &'a RetiredTransfers,
    entries: &'a Mutex<AllEntries>,
}

pub struct Runtime {
    entries: Arc<Mutex<AllEntries>>,
    active_transfers: Arc<ActiveTransfers>,
    retired_transfers: Arc<RetiredTransfers>,
    libusb: Context<Notifier>,
    curr_id: AtomicUsize,
}

impl Runtime {
    pub fn new() -> rusb::Result<Arc<Self>> {
        let entries = Default::default();
        let libusb = Context::with_notifier(Notifier {
            entries: Arc::clone(&entries),
        })?;

        {
            let entries = &mut entries.lock().unwrap().active_and_ready;
            if entries.active_is_empty() {
                entries.extend(libusb.pollfds().into_iter());
            }
        }

        Ok(Arc::new(Runtime {
            entries,
            active_transfers: Default::default(),
            retired_transfers: Default::default(),
            libusb,
            curr_id: AtomicUsize::new(0),
        }))
    }

    pub fn next_id(&self) -> usize {
        self.curr_id.fetch_add(1, Ordering::Relaxed)
    }

    fn _wait_for_all(&self) -> _WaitForAll<'_> {
        unimplemented!()
    }
}

impl Drop for Runtime {
    fn drop(&mut self) {
        // I'd like to implement a way to ensure that
        // all transfers are completed before dropping
        // the context and the remaining transfers themselves.
        //
        // Right now the context doesn't keep track of
        // "dropped" transfers, nor does it have a way to
        // wait for the USB device to finish writing or reading
        // data.

        // At this point we'll just leak the memory. TODO.
        for _ in self
            .retired_transfers
            .inner
            .lock()
            .unwrap()
            .drain(..)
            .map(ManuallyDrop::new)
        {}
    }
}

struct Submission(Submit<LibusbPoll>);

impl Future for Submission {
    type Output = io::Result<LibusbPoll>;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        Pin::new(&mut self.get_mut().0)
            .poll(cx)
            .map(|BufResult(result, poll)| match result {
                Ok(_) => Ok(poll),
                Err(err) => Err(err),
            })
    }
}

fn submit_entry(poll: LibusbPoll) -> Submission {
    Submission(compio_runtime::submit(poll))
}

#[derive(Default)]
struct Entries {
    active: Vec<Submission>,
    ready: Vec<(usize, io::Result<LibusbPoll>)>,
}

impl Entries {
    fn split_borrow_mut(
        &mut self,
    ) -> (
        &mut Vec<Submission>,
        &mut Vec<(usize, io::Result<LibusbPoll>)>,
    ) {
        (&mut self.active, &mut self.ready)
    }

    pub fn submit(&mut self, pollfd: LibusbPoll) {
        self.active.push(submit_entry(pollfd));
    }

    pub fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = LibusbPoll>,
    {
        let entries = iter.into_iter().map(submit_entry);
        self.active.extend(entries);
    }

    pub fn active_len(&self) -> usize {
        self.active.len()
    }

    pub fn active_is_empty(&self) -> bool {
        self.active.is_empty()
    }
}

impl Future for Entries {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Self::Output> {
        let (active, ready) = self.split_borrow_mut();

        let completed =
            active
                .iter_mut()
                .enumerate()
                .filter_map(|(i, s)| match Pin::new(s).poll(cx) {
                    Poll::Ready(e) => Some((i, e)),
                    Poll::Pending => None,
                });

        let start_idx = ready.len();
        ready.extend(completed);

        if let Some(ready) = ready.get(start_idx..) {
            for &(idx, _) in ready.iter().rev() {
                active.swap_remove(idx);
            }
        }

        if ready.is_empty() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

pin_project! {
    pub struct Wait3<'a> {
        #[pin]
        cancel: &'a mut crate::CancellationToken,
        cx: &'a Runtime,
        transfer: &'a crate::LibusbTransfer2,
        id: usize,
    }

    impl PinnedDrop for Wait3<'_> {
        fn drop(this: Pin<&mut Self>) {
            this.cx.active_transfers.remove(this.id);
        }
    }
}

impl Future for Wait3<'_> {
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

        // The transfer's callback function will wake our future up
        // when the transfer has fully completed. The context's
        // transfer list functions as a waker for the file descriptors
        // by choosing any of the available futures to wake up for
        // the purpose of calling into libusb when there are events.
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
                    // As an optimization, we don't call cancel more than
                    // once if we get a cancel signal.
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
                        let entries = &mut this.cx.entries.lock().unwrap().active_and_ready;
                        ready!(Pin::new(entries).poll(&mut cx))
                    }

                    // We don't get to the code below unless we got events
                    // from the above call to `poll`. Otherwise, we would've returned
                    // early with `Poll::Pending`.
                    //
                    // Since our future got the events, we get to call into libusb
                    // to handle those events. Afterwards we must mark that we've handled
                    // the events (for more info on that, see the `PinnedDrop` impl)
                    this.cx
                        .libusb
                        .handle_events()
                        .expect("we should be the only one handling events, and we should only handle events when we get stuff from poll");
                    this.cx.active_transfers.handled_events();

                    // After calling into libusb, we can check which file descriptors
                    // gave us events. If any descriptors were placed into the
                    // `retired` list, then we should not submit those descriptors
                    // again. Otherwise, it's probably okay to submit them again.
                    let mut entries = this.cx.entries.lock().unwrap();
                    entries.update();
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

pub struct Transfer3 {
    context: Arc<Runtime>,
    inner: Option<crate::LibusbTransfer2>,
}

impl Transfer3 {
    pub fn wait<'a>(&'a self, cancel: &'a mut crate::CancellationToken) -> Wait3<'a> {
        Wait3 {
            cancel,
            id: self.context.next_id(),
            cx: &self.context,
            transfer: self.inner.as_ref().unwrap(),
        }
    }
}

fn is_transfer_running(transfer: &mut crate::LibusbTransfer2) -> bool {
    StateRepr::Running == transfer.footer().state().get()
}

impl Drop for Transfer3 {
    fn drop(&mut self) {
        if let Some(transfer) = self.inner.take_if(is_transfer_running) {
            _ = transfer.cancel();
            // Add to context
        }
    }
}

// fn _foo() {
//     let retired_entries = Default::default();
//     let active_entries = Default::default();
//     let libusb = Context::with_notifier(Notifier {
//         retired: Arc::clone(&retired_entries),
//         new: Arc::clone(&active_entries),
//     })
//     .unwrap();
//     let compio = compio_runtime::Runtime::new().unwrap();

//     compio.block_on(async {
//         // We get this list of entries to poll, but how should we
//         // poll them? Ideally, this problem can be solved using a
//         // trait I felt like I saw in the "futures" crate, where
//         // we poll the list until one returns a `Ready`, then swap
//         // out that future and replace it with a new one from the
//         // same pollfd.
//         {
//             let mut entries = active_entries.lock().unwrap();
//             if entries.active_is_empty() {
//                 entries.extend(libusb.pollfds());
//             }
//         }

//         let context = Arc::new(Runtime {
//             active_entries,
//             retired_entries,
//             active_transfers: Arc::new(ActiveTransfers {
//                 inner: RwLock::new(vec![]),
//             }),
//             libusb,
//             curr_id: AtomicUsize::new(0),
//         });

//         // WRONG EXAMPLE. Correct example below this block.
//         let _fut = std::future::poll_fn(|cx| {
//             let mut entries = context.active_entries.lock().unwrap();

//             // See here how we use the context given to us?
//             // I don't really like that.
//             //
//             // I want to use our own context, which stores
//             // a bag of wakers corresponding to unique transfer
//             // IDs, so that when the waker is called, we can
//             // dynamically select an active/valid transfer
//             // future to wake up.
//             //
//             // What happens if our future isn't the one called
//             // to handle the events? As long as we are a transfer
//             // future, then our waker is also going to be stored
//             // in the callback function of the transfer itself.
//             // When another future handles the pending events, libusb
//             // will then run all the applicable callback functions,
//             // including this one, and by the time we poll ourselves,
//             // our transfer will be complete and we don't need to
//             // check here.
//             Pin::new(entries.deref_mut()).poll(cx)
//         });

//         let id = context.curr_id.fetch_add(1, Ordering::Relaxed);
//         std::future::poll_fn(|cx| {
//             // We create an entry that IDs our transfer
//             // and saves our waker in the bag of wakers.
//             // This ID only needs to exist for the length of
//             // the transfer future itself, not for the length
//             // of the transfer as a whole.
//             context.active_transfers.push_or_update(id, cx.waker());

//             // We then create a context using the list of
//             // active transfers.
//             let main_waker = context.active_transfers.clone().into_waker();
//             let mut cx = std::task::Context::from_waker(&main_waker);

//             // Now we can poll all the entries for events.
//             let mut entries = context.active_entries.lock().unwrap();
//             Pin::new(entries.deref_mut()).poll(&mut cx)
//         })
//         .await;

//         // From here, if our transfer isn't complete,
//         // we get to call `libusb_handle_events`.
//         context.libusb.handle_events().unwrap();
//         context.active_transfers.handled_events();

//         // Afterwards we resubmit the entry as long as
//         // it's not listed in a "denylist" managed by
//         // the libusb callback functions.

//         {
//             let mut active = context.active_entries.lock().unwrap();
//             let mut retired = context.retired_entries.lock().unwrap();

//             active.handle_ready(|result| {
//                 let pollfd = result.expect("wonder how this happened?");

//                 if let Some(idx) = retired.iter().position(|&fd| pollfd.fd == fd) {
//                     retired.swap_remove(idx);
//                     None
//                 } else {
//                     Some(pollfd)
//                 }
//             });
//         }

//         // Then we can check if our own transfer is complete.
//         // If so then we can remove our waker and handle
//         // our completed transfer, or wait again.
//         //
//         // (Pretend we're done and can remove the waker)
//         context.active_transfers.remove(id);

//         // There is another scenario we haven't mentioned, and it's
//         // when a transfer future is dropped after having been woken
//         // but before it handled the events. To address this we
//         // need `Drop` code. TODO
//         //
//         // When dropping a transfer future, we need to remove ourselves
//         // from the bag of wakers, but if we were called to handle the events,
//         // then we pass on the responsibility to the next waker in the bag.
//         // Surely this will work.

//         // let future = compio_runtime::submit(entry);

//         // What did we just do here?
//         // I created a compio runtime and a libusb context, then
//         // retrieved the poll descriptors from the context.
//         // For each pollfd, I convert them into a compio OpCode,
//         // then submit them to the runtime, which gives me back
//         // a future to await. After doing this to each pollfd,
//         // I'll now have a set of futures that I need to await
//         // concurrently.
//         //
//         // The first question is "who awaits these futures?"
//         //
//         // Ideally, the first transfer submitted by the program
//         // will be in charge of submitting these file descriptors,
//         // and then the job of polling the returned futures will be shared
//         // by all the subsequent transfers.
//         //
//         // Upon completion of this future,
//         // I know that I am free to call the `handle_events` function
//         // from libusb. I will continue to poll these descriptors until
//         // libusb tells me to stop.
//     });

//     // driver.update_waker(op, waker);

//     // GOAL: Define a paradigm where I can:
//     // - Manage multiple devices with one "context/runtime"
//     // - Have transfers be tied to the lifetime of a device
//     //   (cleanup unfinished transfers)
//     // - Not require an extra async/sync task to manage
//     //   the libusb events
//     //
//     // Ideally, I'd like transfers to kind-of contain everything
//     // they need to run, packing the device and context within
//     // the transfer struct. Each transfer submits itself, then
//     // waits until libusb wakes up the transfer future through the
//     // callback function.
//     // However, who waits for libusb? Who does compio wake up when
//     // one of the pollfds returns with an event?
//     //
//     // 3/27/25: Transfers need an external task that polls for libusb
//     // events, then calls the handle function to wake up the completed
//     // futures. Polling externally prevents us from having to wake up
//     // potentially N futures for every transfer that completes, aka
//     // O(N^2) operation.
//     //
//     // 4/1/25: Silly idea
//     //
//     // What if I made each transfer fight to be the one the async
//     // executor wakes up when a poll returns an event??
//     //
//     // 4/21/25: Continuing on this idea:
//     //
//     // waiters: A simple list (Vec) of Wakers.
//     // - how will it be used?
//     //   - Futures will add their own waker to the list
//     //   - On wake, the first future to obtain a permit will
//     //     call libusb_handle_events and check if their transfer
//     //     is complete
//     //   - If not complete, reenter the queue and await a wakeup
//     //   - If complete, remove self from the queue and return result
//     //
// }
