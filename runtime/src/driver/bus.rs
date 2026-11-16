//! The serial bus: a register-level view of the servos, with retries and timing.

use std::collections::BTreeMap;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::time::{Duration, Instant};

use anyhow::{Context, anyhow, bail, ensure};
use hdrhistogram::Histogram;
use rustypot::DynamixelProtocolHandler;
use rustypot::bus::Bus;
use rustypot::servo::feetech::sts3215::{DEFINITION, REGISTERS};
use rustypot::servo::{Encoding, RegisterInfo};
use serialport::SerialPort;

use crate::telemetry::Reconnection;

pub(super) const BAUD_RATE: u32 = 1_000_000;
/// Longer than a USB serial adapter's 16 ms latency timer, as `LeRobot` uses.
/// A whole state reply is about 1 ms on the wire at 1 Mbaud (six motors, 16
/// bytes each at 10 us a byte), so only the adapter's latency timer keeps this
/// from being shorter. Shorten it only from the `BusTiming` histogram of the
/// adapter in use.
pub(super) const TIMEOUT: Duration = Duration::from_millis(50);
/// A read whose reply is lost or corrupted is asked again this many times before
/// the bus counts as failed; one dropped packet should not end a motion.
pub(super) const READ_RETRIES: u32 = 1;
pub(super) const WRITE_RETRIES: u32 = 1;
pub(super) const TORQUE_RETRIES: u32 = 3;

/// The registers of the servos on one bus, by name. A bus that fails cannot be
/// recovered from inside the loop, so failures are opaque errors with context.
pub trait ServoBus {
    /// The live registers of every motor in `ids`, in one transaction.
    ///
    /// # Errors
    /// The bus did not answer after retries.
    fn read_state_block(&mut self, ids: &[u8]) -> anyhow::Result<Vec<StateBlock>>;
    /// # Errors
    /// The bus did not answer after retries, or a motor refused the read.
    fn read(&mut self, ids: &[u8], register: &str) -> anyhow::Result<Vec<i64>>;
    /// # Errors
    /// The bus did not take the write, after retries.
    fn write(&mut self, ids: &[u8], register: &str, values: &[i64]) -> anyhow::Result<()>;
    /// # Errors
    /// A servo did not take the torque change, after retries.
    fn set_torque(&mut self, ids: &[u8], enabled: bool) -> anyhow::Result<()>;
    /// Get a lost bus back without blocking: the port is closed, and an attempt
    /// to open it again by its configured path starts when none runs and the
    /// last began [`RECONNECT_INTERVAL`] ago. Polled each cycle until restored.
    fn reconnect(&mut self) -> Reconnection;
}

/// Whether two register names are equal; `==` on strings is not usable in a
/// `const`.
const fn same_name(first: &str, second: &str) -> bool {
    let (first, second) = (first.as_bytes(), second.as_bytes());
    if first.len() != second.len() {
        return false;
    }
    let mut index = 0;
    while index < first.len() {
        if first[index] != second[index] {
            return false;
        }
        index += 1;
    }
    true
}

/// A register of the STS3215's control table, looked up while compiling, so the
/// state block below cannot disagree with the table.
#[allow(
    clippy::panic,
    reason = "Only constant register names call this function; invalid names fail compilation"
)]
const fn register(name: &str) -> RegisterInfo {
    let mut index = 0;
    while index < REGISTERS.len() {
        if same_name(REGISTERS[index].name, name) {
            return REGISTERS[index];
        }
        index += 1;
    }
    panic!("the STS3215 table has no such register")
}

/// First register of the contiguous live block: `Present_Position`.
pub(super) const STATE_BLOCK_ADDRESS: u8 = register("present_position").addr;
/// From `Present_Position` through `Status` inclusive: position, speed (not
/// decoded: the loop estimates velocity from positions), load, voltage,
/// temperature, the asynchronous-write flag, and status.
pub(super) const STATE_BLOCK_LENGTH: u8 = {
    let status = register("status");
    status.addr + status.size - STATE_BLOCK_ADDRESS
};

/// Where a register's bytes start within the state block.
const fn block_offset(name: &str) -> usize {
    (register(name).addr - STATE_BLOCK_ADDRESS) as usize
}

/// One motor's live registers, decoded as the servo reports them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StateBlock {
    /// Encoder steps.
    pub position: i64,
    /// PWM duty in tenths of a percent, signed by direction.
    pub load: i64,
    /// Tenths of a volt.
    pub voltage: i64,
    /// Degrees Celsius.
    pub temperature: i64,
    /// Feetech alarm bits.
    pub status: i64,
    /// Error bits returned in the protocol status packet.
    pub packet_status: u8,
}

impl StateBlock {
    /// Decode registers 56..=65, whose layout is in the STS3215 table.
    pub(super) fn decode(bytes: &[u8]) -> anyhow::Result<Self> {
        ensure!(
            bytes.len() == usize::from(STATE_BLOCK_LENGTH),
            "A state block has {} bytes, expected {STATE_BLOCK_LENGTH}",
            bytes.len()
        );
        let word =
            |offset: usize| u64::from(u16::from_le_bytes([bytes[offset], bytes[offset + 1]]));
        Ok(Self {
            position: Encoding::SignMagnitude { sign_bit: 15 }
                .decode(word(block_offset("present_position")), 2),
            load: Encoding::SignMagnitude { sign_bit: 10 }
                .decode(word(block_offset("present_load")), 2),
            voltage: i64::from(bytes[block_offset("present_voltage")]),
            temperature: i64::from(bytes[block_offset("present_temperature")]),
            status: i64::from(bytes[block_offset("status")]),
            packet_status: 0,
        })
    }
}

/// How the bus has answered so far: what decides how short `TIMEOUT` can be.
/// A good reply must arrive well inside it, or it is mistaken for a lost one.
pub struct BusTiming {
    /// Each answered attempt's round trip, in microseconds.
    pub answered_round_trip_microseconds: Histogram<u64>,
    /// Attempts beyond the first: replies lost, corrupted or late.
    pub retries: u64,
    /// Requests still unanswered after every retry.
    pub failed_requests: u64,
}

impl BusTiming {
    #[allow(
        clippy::expect_used,
        reason = "Fixed positive histogram bounds and three significant digits are valid"
    )]
    fn new() -> Self {
        Self {
            // Up to ten seconds, to three significant digits.
            answered_round_trip_microseconds: Histogram::new_with_bounds(1, 10_000_000, 3)
                .expect("the histogram bounds are valid"),
            retries: 0,
            failed_requests: 0,
        }
    }
}

/// rustypot's own error type; it is not `Send`, so it cannot be an error source.
type RustypotResult<T> = Result<T, Box<dyn std::error::Error>>;

/// How often the port of a lost bus is opened again.
pub(super) const RECONNECT_INTERVAL: Duration = Duration::from_secs(1);

/// An open serial port: rustypot's bus over it, and the same port for the
/// multi-register reads rustypot's `Bus` does not offer.
struct Port {
    bus: Bus,
    block_port: Box<dyn SerialPort>,
}

impl Port {
    /// Open the port by its configured path. Startup and every reconnection open
    /// it here, so a reconnected bus is set up exactly as at startup.
    fn open(path: &str, motor_ids: &[u8]) -> anyhow::Result<Self> {
        let serial = serialport::new(path, BAUD_RATE)
            .timeout(TIMEOUT)
            .open()
            .with_context(|| format!("Cannot open the Feetech bus {path}"))?;
        let block_port = serial
            .try_clone()
            .with_context(|| format!("Cannot share the Feetech bus {path}"))?;
        let motors: BTreeMap<u8, _> = motor_ids.iter().map(|id| (*id, DEFINITION)).collect();
        Ok(Self {
            bus: Bus::new(serial, motors),
            block_port,
        })
    }

    /// Open the port and hear every motor answer: what a lost bus must do to
    /// count as back. Slow while the servos are silent, so it runs off the loop.
    fn open_answering(path: &str, motor_ids: &[u8]) -> anyhow::Result<Self> {
        let mut port = Self::open(path, motor_ids)?;
        let answers = port
            .bus
            .sync_read_register(motor_ids, "torque_enable")
            .map_err(|failure| {
                anyhow!("Feetech bus {path}: motors {motor_ids:?} did not all answer: {failure}")
            })?;
        ensure!(
            answers.len() == motor_ids.len(),
            "Feetech bus {path}: {} of {} motors answered",
            answers.len(),
            motor_ids.len()
        );
        Ok(port)
    }
}

/// A reconnection running on its own thread, so no cycle waits on a port that
/// may take a timeout per silent motor to answer.
struct Attempt {
    started: Instant,
    /// Gone once this attempt's outcome was taken.
    outcome: Option<Receiver<anyhow::Result<Port>>>,
}

/// The real bus: rustypot over a serial port.
pub struct SerialBus {
    path: String,
    motor_ids: Vec<u8>,
    /// None from a reconnection's start until it restores the bus.
    port: Option<Port>,
    attempt: Option<Attempt>,
    protocol: DynamixelProtocolHandler,
    timing: BusTiming,
}

impl SerialBus {
    /// # Errors
    /// The port cannot be opened.
    pub fn open(path: &str, motor_ids: &[u8]) -> anyhow::Result<Self> {
        Ok(Self {
            path: path.to_owned(),
            motor_ids: motor_ids.to_vec(),
            port: Some(Port::open(path, motor_ids)?),
            attempt: None,
            protocol: DynamixelProtocolHandler::v1(),
            timing: BusTiming::new(),
        })
    }

    #[must_use]
    pub fn timing(&self) -> &BusTiming {
        &self.timing
    }

    fn port(&mut self) -> anyhow::Result<&mut Port> {
        self.port
            .as_mut()
            .with_context(|| format!("The Feetech bus {} is disconnected", self.path))
    }

    /// Run one request with retries, timing each answered attempt and counting
    /// the ones that were not.
    fn timed_request<T>(
        bus: &mut Bus,
        timing: &mut BusTiming,
        retries: u32,
        register: &str,
        mut request: impl FnMut(&mut Bus) -> RustypotResult<T>,
    ) -> anyhow::Result<T> {
        let mut attempts = 0_u64;
        let outcome = bus.with_retries(retries, |bus| {
            attempts += 1;
            let sent = Instant::now();
            let answer = request(bus);
            if answer.is_ok() {
                timing.answered_round_trip_microseconds.saturating_record(
                    u64::try_from(sent.elapsed().as_micros()).unwrap_or(u64::MAX),
                );
            }
            answer
        });
        timing.retries += attempts.saturating_sub(1);
        if outcome.is_err() {
            timing.failed_requests += 1;
        }
        outcome.map_err(|failure| {
            crate::telemetry::CommunicationFailure(format!("Feetech bus, {register}: {failure}"))
                .into()
        })
    }
}

impl ServoBus for SerialBus {
    fn read_state_block(&mut self, ids: &[u8]) -> anyhow::Result<Vec<StateBlock>> {
        let Some(Port { bus, block_port }) = self.port.as_mut() else {
            bail!("The Feetech bus {} is disconnected", self.path);
        };
        let protocol = &self.protocol;
        let replies =
            Self::timed_request(bus, &mut self.timing, READ_RETRIES, "state block", |_| {
                protocol.sync_read_with_error(
                    block_port.as_mut(),
                    ids,
                    STATE_BLOCK_ADDRESS,
                    STATE_BLOCK_LENGTH,
                )
            })?;
        replies
            .iter()
            .map(|(bytes, packet_status)| {
                let mut block = StateBlock::decode(bytes)?;
                block.packet_status = packet_status.byte();
                Ok(block)
            })
            .collect()
    }

    fn read(&mut self, ids: &[u8], register: &str) -> anyhow::Result<Vec<i64>> {
        let Some(Port { bus, .. }) = self.port.as_mut() else {
            bail!("The Feetech bus {} is disconnected", self.path);
        };
        // An STS3215's reply status holds alarms only, never a refusal, so the
        // value is all there is to read.
        Self::timed_request(bus, &mut self.timing, READ_RETRIES, register, |bus| {
            bus.sync_read_register(ids, register)
        })
    }

    fn write(&mut self, ids: &[u8], register: &str, values: &[i64]) -> anyhow::Result<()> {
        let Some(Port { bus, .. }) = self.port.as_mut() else {
            bail!("The Feetech bus {} is disconnected", self.path);
        };
        Self::timed_request(bus, &mut self.timing, WRITE_RETRIES, register, |bus| {
            bus.sync_write_register(ids, register, values)
        })
    }

    fn reconnect(&mut self) -> Reconnection {
        // An open port would refuse its own reopening (it is opened exclusively),
        // and after a USB re-enumeration it is dead anyway.
        self.port = None;
        if let Some(attempt) = &mut self.attempt
            && let Some(outcome) = &attempt.outcome
        {
            let finished = match outcome.try_recv() {
                Err(TryRecvError::Empty) => return Reconnection::Waiting,
                Ok(finished) => finished,
                Err(TryRecvError::Disconnected) => Err(anyhow!("The reconnection thread ended")),
            };
            attempt.outcome = None;
            return match finished {
                Ok(port) => {
                    self.port = Some(port);
                    self.attempt = None;
                    Reconnection::Restored
                }
                Err(failure) => Reconnection::Failed(failure),
            };
        }
        if self
            .attempt
            .as_ref()
            .is_none_or(|attempt| attempt.started.elapsed() >= RECONNECT_INTERVAL)
        {
            let (sender, receiver) = mpsc::channel();
            let (path, motor_ids) = (self.path.clone(), self.motor_ids.clone());
            std::thread::spawn(move || {
                let _ = sender.send(Port::open_answering(&path, &motor_ids));
            });
            self.attempt = Some(Attempt {
                started: Instant::now(),
                outcome: Some(receiver),
            });
        }
        Reconnection::Waiting
    }

    fn set_torque(&mut self, ids: &[u8], enabled: bool) -> anyhow::Result<()> {
        let failed = self.port()?.bus.set_torque(ids, enabled, TORQUE_RETRIES);
        if failed.is_empty() {
            return Ok(());
        }
        let motors: Vec<String> = failed
            .iter()
            .map(|(id, failure)| format!("{id}: {failure}"))
            .collect();
        bail!(
            "Torque {} failed: {}",
            if enabled { "enable" } else { "disable" },
            motors.join(", ")
        )
    }
}
