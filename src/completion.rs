use std::{
    future::Future,
    pin::{pin, Pin},
    task::ready,
};

use pin_project_lite::pin_project;
use tokio::sync::{futures::Notified, Notify};
use tokio_util::sync::{CancellationToken, WaitForCancellationFuture};

#[derive(Debug)]
pub enum Event {
    Completed,
    Cancelled,
}

pin_project! {
    pub struct Completion<'a> {
        #[pin]
        complete: Notified<'a>,
        #[pin]
        cancel: WaitForCancellationFuture<'a>,
        is_cancelled: bool,
    }
}

impl<'a> Completion<'a> {
    pub fn new(complete: &'a Notify, cancel: &'a CancellationToken) -> Self {
        Self {
            complete: complete.notified(),
            cancel: cancel.cancelled(),
            is_cancelled: cancel.is_cancelled(),
        }
    }
}

impl Future for Completion<'_> {
    type Output = Event;

    fn poll(
        mut self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        #[inline(always)]
        async fn select(
            complete: Pin<&mut Notified<'_>>,
            cancel: Pin<&mut WaitForCancellationFuture<'_>>,
            is_cancelled: bool,
        ) -> Event {
            tokio::select! {
                biased;
                _ = cancel, if !is_cancelled => Event::Cancelled,
                _ = complete => Event::Completed,
            }
        }

        let this = self.as_mut().project();
        let complete = this.complete;
        let cancel = this.cancel;

        let fut = select(complete, cancel, *this.is_cancelled);
        match ready!(pin!(fut).poll(cx)) {
            Event::Completed => std::task::Poll::Ready(Event::Completed),
            Event::Cancelled => {
                *this.is_cancelled = true;
                std::task::Poll::Ready(Event::Cancelled)
            }
        }
    }
}
