//! Match connected USB controllers to a local SO-101 configuration checkout.
//!
//! Discovery reads USB metadata only. Loading a match never opens the serial bus
//! or changes motor registers. Configuration retrieval/authentication stays with
//! the caller so ordinary startup works offline.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail, ensure};
use serde::Serialize;
use serialport::{SerialPortInfo, SerialPortType};

/// A controller serial, checked before it can become a registry directory name.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(transparent)]
pub struct ControllerSerial(String);

impl ControllerSerial {
    /// # Errors
    /// The serial is empty or contains characters outside a single directory name.
    pub fn parse(serial: &str) -> anyhow::Result<Self> {
        ensure!(
            !serial.is_empty()
                && serial
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_')),
            "USB controller serial must contain only letters, digits, '-' or '_'"
        );
        Ok(Self(serial.to_owned()))
    }

    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// USB metadata for one device. It is not evidence of motor identity or health.
#[derive(Debug, Clone, Serialize)]
pub struct UsbController {
    pub serial: ControllerSerial,
    pub port: String,
}

/// Registry association before any profile/calibration data is trusted.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum ControllerMatch {
    Registered {
        controller: UsbController,
        directory: PathBuf,
    },
    Unregistered {
        controller: UsbController,
    },
    MissingSerial {
        port: String,
    },
    InvalidSerial {
        port: String,
        serial: String,
    },
    AmbiguousSerial {
        serial: ControllerSerial,
        ports: Vec<String>,
    },
}

/// Associate enumerated USB devices with registry entries; duplicate serials
/// never select an arbitrary port. Non-USB devices are ignored.
///
/// # Errors
/// The registry is absent or its arms directory cannot be read.
pub fn match_controllers(
    ports: &[SerialPortInfo],
    config_directory: &Path,
) -> anyhow::Result<Vec<ControllerMatch>> {
    let arms_directory = config_directory.join("arms");
    std::fs::read_dir(&arms_directory)
        .with_context(|| format!("Cannot read SO-101 configs at {}; set --so101-config-directory to a local config checkout", arms_directory.display()))?;
    let mut grouped: BTreeMap<ControllerSerial, Vec<String>> = BTreeMap::new();
    let mut matches = Vec::new();
    for port in ports {
        let SerialPortType::UsbPort(info) = &port.port_type else {
            continue;
        };
        let Some(serial) = info
            .serial_number
            .as_deref()
            .filter(|serial| !serial.is_empty())
        else {
            matches.push(ControllerMatch::MissingSerial {
                port: port.port_name.clone(),
            });
            continue;
        };
        match ControllerSerial::parse(serial) {
            Ok(serial) => grouped
                .entry(serial)
                .or_default()
                .push(port.port_name.clone()),
            Err(_) => matches.push(ControllerMatch::InvalidSerial {
                port: port.port_name.clone(),
                serial: serial.to_owned(),
            }),
        }
    }
    for (serial, mut ports) in grouped {
        ports.sort();
        ports.dedup();
        // macOS exposes the same serial device as both a callout and a dial-in
        // node. Prefer the callout node, but keep other duplicate serials ambiguous.
        #[cfg(target_os = "macos")]
        {
            let callout_ports = ports.clone();
            ports.retain(|port| {
                port.strip_prefix("/dev/tty.")
                    .is_none_or(|suffix| !callout_ports.contains(&format!("/dev/cu.{suffix}")))
            });
        }
        let directory = arms_directory.join(serial.as_str());
        if !directory.is_dir() {
            matches.extend(ports.into_iter().map(|port| ControllerMatch::Unregistered {
                controller: UsbController {
                    serial: serial.clone(),
                    port,
                },
            }));
            continue;
        }
        if ports.len() > 1 {
            matches.push(ControllerMatch::AmbiguousSerial { serial, ports });
            continue;
        }
        let controller = UsbController {
            serial,
            port: ports.remove(0),
        };
        matches.push(ControllerMatch::Registered {
            controller,
            directory,
        });
    }
    Ok(matches)
}

/// Enumerate USB metadata without opening any serial device.
///
/// # Errors
/// USB enumeration or the registry cannot be read.
pub fn discover(config_directory: &Path) -> anyhow::Result<Vec<ControllerMatch>> {
    match_controllers(
        &serialport::available_ports().context("Cannot enumerate serial ports")?,
        config_directory,
    )
}

/// A uniquely selected connected controller and its registry entry.
#[derive(Debug)]
pub struct RegisteredController {
    pub controller: UsbController,
    pub directory: PathBuf,
}

/// Select the requested serial, or the only registered controller.
///
/// # Errors
/// No controller matches, the requested serial is unknown/ambiguous, or several
/// registered controllers are connected without an explicit selection.
pub fn select_controller(
    matches: &[ControllerMatch],
    requested: Option<&ControllerSerial>,
) -> anyhow::Result<RegisteredController> {
    for candidate in matches {
        if let ControllerMatch::AmbiguousSerial { serial, ports } = candidate
            && requested.is_none_or(|requested| requested == serial)
        {
            bail!(
                "USB serial {} has multiple ports {ports:?}; cannot identify one arm",
                serial.as_str()
            );
        }
    }
    let candidates: Vec<_> = matches
        .iter()
        .filter_map(|candidate| match candidate {
            ControllerMatch::Registered {
                controller,
                directory,
            } if requested.is_none_or(|requested| requested == &controller.serial) => {
                Some((controller, directory))
            }
            _ => None,
        })
        .collect();
    match candidates.as_slice() {
        [(controller, directory)] => Ok(RegisteredController {
            controller: (*controller).clone(),
            directory: (*directory).clone(),
        }),
        [] => bail!(
            "No registered SO-101 controller matches {}; run 'so101-discover' to inspect connected USB devices",
            requested.map_or("the connected devices", ControllerSerial::as_str)
        ),
        _ => bail!(
            "Multiple registered SO-101 controllers are connected; select one with --so101-serial: {}",
            candidates
                .iter()
                .map(|(controller, _)| controller.serial.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    reason = "Test assertions fail immediately on unexpected outcomes"
)]
mod tests;
