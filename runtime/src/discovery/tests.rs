use super::*;
use serialport::UsbPortInfo;

struct RegistryFixture(PathBuf);
impl RegistryFixture {
    fn new() -> Self {
        let directory =
            std::env::temp_dir().join(format!("so101-discovery-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(directory.join("arms")).unwrap();
        Self(directory)
    }
    fn register(&self, serial: &str) {
        std::fs::create_dir_all(self.0.join("arms").join(serial)).unwrap();
    }
}
impl Drop for RegistryFixture {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

fn port(name: &str, serial: Option<&str>) -> SerialPortInfo {
    SerialPortInfo {
        port_name: name.to_owned(),
        port_type: SerialPortType::UsbPort(UsbPortInfo {
            vid: 0x1234,
            pid: 0x5678,
            serial_number: serial.map(str::to_owned),
            manufacturer: None,
            product: None,
        }),
    }
}

#[test]
fn selecting_among_multiple_arms_requires_a_serial_and_rejects_duplicate_serials() {
    let fixture = RegistryFixture::new();
    fixture.register("ARM_A");
    fixture.register("ARM_B");
    let ports = [
        port("/dev/ttyUSB0", Some("ARM_A")),
        port("/dev/ttyUSB1", Some("ARM_B")),
    ];
    let matches = match_controllers(&ports, &fixture.0).unwrap();
    assert!(
        select_controller(&matches, None)
            .unwrap_err()
            .to_string()
            .contains("Multiple registered")
    );
    let requested = ControllerSerial::parse("ARM_B").unwrap();
    assert_eq!(
        select_controller(&matches, Some(&requested))
            .unwrap()
            .controller
            .port,
        "/dev/ttyUSB1"
    );
    let duplicates = [
        port("/dev/ttyUSB0", Some("ARM_A")),
        port("/dev/ttyUSB2", Some("ARM_A")),
    ];
    let matches = match_controllers(&duplicates, &fixture.0).unwrap();
    assert!(select_controller(&matches, None).is_err());
    assert!(select_controller(&matches, Some(&ControllerSerial::parse("ARM_A").unwrap())).is_err());
}

#[test]
fn unknown_missing_and_unsafe_serials_never_select_a_configuration() {
    let fixture = RegistryFixture::new();
    let matches = match_controllers(
        &[
            port("/dev/a", Some("UNKNOWN")),
            port("/dev/b", None),
            port("/dev/c", Some("../elsewhere")),
        ],
        &fixture.0,
    )
    .unwrap();
    assert!(
        matches
            .iter()
            .any(|candidate| matches!(candidate, ControllerMatch::Unregistered { .. }))
    );
    assert!(
        matches
            .iter()
            .any(|candidate| matches!(candidate, ControllerMatch::MissingSerial { .. }))
    );
    assert!(
        matches
            .iter()
            .any(|candidate| matches!(candidate, ControllerMatch::InvalidSerial { .. }))
    );
    assert!(select_controller(&matches, None).is_err());
    assert!(ControllerSerial::parse("").is_err());
}
