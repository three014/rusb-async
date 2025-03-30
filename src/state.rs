use std::sync::atomic::{AtomicU8, Ordering};

const RUNNING: u8 = 0b1;
const IDLE: u8 = 0b0;
const COMPLETE: u8 = 0b10;
const FINISHED: u8 = 0b10000000;
const HELPED: u8 = 0b01000000;

#[inline(always)]
const fn is_helping(val: u8) -> bool {
    val & HELPED == HELPED
}

#[inline(always)]
const fn is_finishing(val: u8) -> bool {
    val & FINISHED == FINISHED
}

/// The state of a libusb transfer.
///
/// # The lifetime of a state
///
/// There are at most two threads handling a transfer and therefore
/// a state object. Thread A is the one that submitted the transfer
/// and Thread B is the one running the callback function once a
/// transfer is completed.
///
/// Thread A sets the state to RUNNING before submitting the
/// transfer, and then sets the state to back to IDLE if
/// the submission failed.
///
/// Thread A waits on either a cancellation or a notification
/// that the transfer is complete.
/// Upon receiving a notification, we know that we can go ahead
/// and mess with our transfer.
///
/// So then where's the need for a state if we already have a
/// notifier?
///
/// The issue is when we drop our future and need to restart it,
/// or when we're about to drop our transfer container while
/// libusb still has the transfer.
///
///
#[derive(Debug)]
pub struct State {
    inner: AtomicU8,
}

impl State {
    #[inline]
    pub const fn new_idle() -> Self {
        Self {
            inner: AtomicU8::new(IDLE),
        }
    }

    /// Sets the state to "running". Returns
    /// `true` if the state was set successfully,
    /// and `false` if the state was in the process
    /// of being set to "completed".
    #[inline]
    pub fn set_running(&self) -> bool {
        !is_finishing(self.inner.fetch_or(RUNNING, Ordering::AcqRel))
    }

    #[inline]
    pub fn set_ready(&self) -> bool {
        let prev_state = self.inner.fetch_or(COMPLETE, Ordering::AcqRel);
        let prev_state = match prev_state {
            RUNNING => RUNNING | COMPLETE,
            IDLE => COMPLETE,
            _ => return false,
        };
        let val = match self.inner.compare_exchange(
            prev_state,
            FINISHED,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => return true,
            Err(val) => val,
        };
        is_helping(val) && is_helping(self.inner.swap(FINISHED, Ordering::AcqRel))
    }

    #[inline]
    pub fn get(&self) -> StateRepr {
        let state = self.inner.load(Ordering::Acquire);

        // When checking for the completed state,
        // we don't care if we had come from an
        // idle nor running state.
        let val = if state & 0b11111110 == COMPLETE {
            match self.inner.compare_exchange(
                COMPLETE,
                FINISHED | HELPED,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return StateRepr::Ready,
                Err(val) => val,
            }
        } else {
            state
        };

        match val {
            val if is_finishing(val) => StateRepr::Ready,
            val if val == COMPLETE + RUNNING => StateRepr::Ready,
            RUNNING => StateRepr::Running,
            IDLE => StateRepr::Idle,
            bad => invalid_value(bad),
        }
    }
}

impl Default for State {
    fn default() -> Self {
        State::new_idle()
    }
}

#[inline(never)]
#[cold]
fn invalid_value(val: u8) -> ! {
    unreachable!("unaccounted value: {val}")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateRepr {
    Idle,
    Running,
    Ready,
}

#[cfg(test)]
mod tests {
    use std::{sync::Arc, time::Duration};

    use super::*;

    #[test]
    fn new_state_is_idle() {
        let state = State::new_idle();
        assert_eq!(StateRepr::Idle, state.get());
    }

    #[test]
    fn st_set_running() {
        let state = State::new_idle();
        assert!(state.set_running());
    }

    #[test]
    fn st_is_running() {
        let state = State::new_idle();
        assert!(state.set_running());
        assert_eq!(StateRepr::Running, state.get());
    }

    #[test]
    fn st_is_finished() {
        let state = State::new_idle();
        assert!(state.set_running());
        assert!(state.set_ready());
        assert_eq!(StateRepr::Ready, state.get());
    }

    #[test]
    fn st_cannot_change_after_ready() {
        let state = State::new_idle();
        assert!(state.set_running());
        assert!(state.set_ready());
        assert!(!state.set_running());
        assert_eq!(StateRepr::Ready, state.get());
    }

    #[test]
    fn st_can_set_ready_after_idle() {
        let state = State::new_idle();
        assert!(state.set_ready());
        assert_eq!(StateRepr::Ready, state.get());
    }

    #[test]
    fn st_cannot_data_race() {
        let state = Arc::new(State::new_idle());
        let state2 = Arc::clone(&state);

        std::thread::scope(|s| {
            let handle = s.spawn(|| {
                std::thread::sleep(Duration::from_micros(500));
                assert!(state2.set_ready());
            });
            assert!(state.set_running());
            handle.join().unwrap();
        });

        assert_eq!(StateRepr::Ready, state.get());
    }
}
