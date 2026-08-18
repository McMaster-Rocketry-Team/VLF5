pub mod armed_mode;
pub mod landed_mode;
pub mod low_power_mode;
pub mod self_test_mode;
pub mod demo_mode;

use core::cell::Cell;

use embassy_time::{Duration, Instant};

/// How long a CAN status stream may be silent before the downlink fields it is
/// the *only* source for are cleared.
///
/// Those fields are written exclusively from a `CanBusMessageEnum` match arm,
/// so with nothing to clear them the last value received is transmitted for
/// ever after the sender stops — a battery voltage taken minutes ago, still
/// reading as a healthy pack.
///
/// Deliberately NOT `CanNode::is_online()`. `last_received_time_us` is written
/// only from the `NodeStatus` handler, so `is_online()` tracks the *heartbeat*
/// and structurally cannot say whether the *status stream* is alive. Both
/// senders prove the difference: AMP's status message comes from a task
/// independent of its heartbeat, and Icarus's `IcarusStatus` comes from its
/// servo control loop — a loop that has stalled, or bailed out to re-home
/// after a run of UART errors, stops reporting while the heartbeat keeps
/// claiming Healthy/Operational. Hence a receipt timestamp per stream,
/// recorded where the message actually arrives.
///
/// 5 s, matching the `is_online()` window so the two freshness answers on a
/// downlink packet are on the same timebase. Against the actual cadences it is
/// generous in the right direction — AMP sends `AmpStatusMessage` every 500 ms
/// (10 missed frames) and Icarus sends `IcarusStatusMessage` at 100 Hz
/// (500 missed reports) — so a burst of arbitration losses cannot blank a
/// working field, while a sender that has genuinely stopped is caught within
/// one 2 s telemetry period of the packet after the window closes.
///
/// The timekeeping stays here rather than in `firmware-common-new`: that crate
/// is shared with the host build and has no business knowing about
/// `embassy_time::Instant`.
pub const CAN_STATUS_STALE_AFTER: Duration = Duration::from_secs(5);

/// When one CAN status stream last delivered a message.
///
/// A `Cell` because the writer (the mode's CAN receive future) and the reader
/// (the mode's periodic packet-building loop) are two futures joined onto one
/// `NoopRawMutex` executor — they never run concurrently, so a lock would buy
/// nothing.
pub type StatusStreamReceipt = Cell<Option<Instant>>;

/// Record that this status stream just delivered a message.
pub fn mark_status_received(receipt: &StatusStreamReceipt) {
    receipt.set(Some(Instant::now()));
}

/// Whether this stream has been silent long enough that the fields it feeds no
/// longer describe anything.
///
/// A stream that has never delivered is stale, which is the same statement:
/// there is no report to relay. That also means the packet builders' initial
/// values for these fields — `PowerOutputStatus::Disabled` for the AMP outputs,
/// which is a claim AMP never made — are overwritten with the honest absence on
/// the first periodic tick rather than surviving until the first message.
pub fn status_stream_stale(receipt: &StatusStreamReceipt) -> bool {
    match receipt.get() {
        Some(received_at) => {
            Instant::now().saturating_duration_since(received_at) > CAN_STATUS_STALE_AFTER
        }
        None => true,
    }
}
