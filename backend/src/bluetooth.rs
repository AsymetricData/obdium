//! Bluetooth Classic (Serial Port Profile) adapter support.
//!
//! ELM327 adapters using Bluetooth Classic expose their AT command channel
//! through SPP. On Linux the kernel exposes that as an RFCOMM socket
//! (`AF_BLUETOOTH` / `BTPROTO_RFCOMM`), so we talk to it directly instead of
//! going through `rfcomm bind` and `/dev/rfcommN`. That means:
//!
//! * no root privileges required (BlueZ handles pairing/authentication),
//! * no dependency on the deprecated `bluez-deprecated` tools,
//! * the adapter shows up in the regular port list as `BT:AA:BB:CC:DD:EE:FF`.
//!
//! Only paired devices advertising the SPP service UUID are listed.
//! BLE (GATT) adapters are intentionally not supported here.

#![cfg(target_os = "linux")]

use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::os::unix::io::RawFd;
use std::process::Command;
use std::sync::Mutex;
use std::time::Duration;

/// Prefix used to distinguish Bluetooth ports from regular serial paths.
pub const PORT_PREFIX: &str = "BT:";

/// SPP service UUID. Present in `bluetoothctl info` output for paired
/// adapters that expose a serial port.
const SPP_UUID: &str = "00001101-0000-1000-8000-00805f9b34fb";

/// Classic serial adapters conventionally listen on channel 1. Clones are not
/// always consistent, so a few channels are probed at connect time.
const RFCOMM_CHANNELS: std::ops::RangeInclusive<u8> = 1..=8;

const CONNECT_TIMEOUT: Duration = Duration::from_millis(2500);
const PROBE_TIMEOUT: Duration = Duration::from_millis(800);

const AF_BLUETOOTH: libc::c_int = 31;
const BTPROTO_RFCOMM: libc::c_int = 3;

/// Remembers the RFCOMM channel that worked for a given adapter so we only
/// probe once per device.
static CHANNEL_CACHE: Mutex<Option<HashMap<String, u8>>> = Mutex::new(None);

#[repr(C)]
struct SockAddrRc {
    rc_family: u16,
    rc_bdaddr: [u8; 6],
    rc_channel: u8,
}

/// A paired Bluetooth Classic adapter exposing a serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BluetoothDevice {
    pub name: String,
    pub address: String,
}

impl BluetoothDevice {
    /// Port identifier understood by [`is_bluetooth_port`] and passed back to
    /// the frontend in the regular serial port list.
    pub fn port_name(&self) -> String {
        format!("{PORT_PREFIX}{}", self.address)
    }
}

/// Whether `port` refers to a Bluetooth adapter rather than a serial device.
pub fn is_bluetooth_port(port: &str) -> bool {
    parse_mac(port).is_some()
}

/// Extract the MAC address from a `BT:AA:BB:CC:DD:EE:FF` port identifier.
pub fn parse_mac(port: &str) -> Option<String> {
    let rest = port.strip_prefix(PORT_PREFIX)?;
    let octets: Vec<&str> = rest.split(':').collect();
    if octets.len() != 6 {
        return None;
    }

    for octet in &octets {
        if octet.len() != 2 || u8::from_str_radix(octet, 16).is_err() {
            return None;
        }
    }

    Some(rest.to_ascii_uppercase())
}

/// List paired adapters that advertise the Serial Port Profile.
///
/// Returns an empty list when BlueZ is unavailable, so the app keeps working
/// exactly as before on machines without Bluetooth.
pub fn list_devices() -> Vec<BluetoothDevice> {
    let output = match Command::new("bluetoothctl").arg("devices").output() {
        Ok(output) if output.status.success() => {
            String::from_utf8_lossy(&output.stdout).into_owned()
        }
        _ => return Vec::new(),
    };

    let candidates = parse_device_list(&output);
    let mut devices = Vec::new();

    for (address, name) in candidates {
        let info = match Command::new("bluetoothctl").arg("info").arg(&address).output() {
            Ok(output) if output.status.success() => {
                String::from_utf8_lossy(&output.stdout).into_owned()
            }
            _ => continue,
        };

        if has_spp_service(&info) {
            devices.push(BluetoothDevice { name, address });
        }
    }

    devices
}

/// Parse `bluetoothctl devices` output:
/// `Device AA:BB:CC:DD:EE:FF Some name`
fn parse_device_list(output: &str) -> Vec<(String, String)> {
    output
        .lines()
        .filter_map(|line| {
            let rest = line.trim().strip_prefix("Device ")?;
            let (address, name) = rest.split_once(' ').unwrap_or((rest, ""));
            parse_mac(&format!("{PORT_PREFIX}{address}"))?;
            Some((address.to_ascii_uppercase(), name.trim().to_string()))
        })
        .collect()
}

/// Whether `bluetoothctl info` output mentions the SPP service.
fn has_spp_service(info: &str) -> bool {
    info.lines().any(|line| {
        let line = line.to_ascii_lowercase();
        line.contains(SPP_UUID) || line.contains("serial port")
    })
}

/// Open an RFCOMM connection to `port` (`BT:AA:BB:CC:DD:EE:FF`).
///
/// The RFCOMM channel is discovered by probing: a channel is considered valid
/// when the adapter answers an `AT` probe with the ELM327 prompt (`>`).
pub fn connect(port: &str, baud_rate: u32) -> io::Result<BluetoothSerialPort> {
    let mac = parse_mac(port)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "invalid Bluetooth port"))?;

    let mut last_error = io::Error::new(io::ErrorKind::NotFound, "adapter did not answer AT probe");

    for channel in channels_for(&mac) {
        let fd = match open_rfcomm(&mac, channel) {
            Ok(fd) => fd,
            Err(error) => {
                last_error = error;
                continue;
            }
        };

        let mut stream = BluetoothSerialPort {
            fd,
            mac: mac.clone(),
            channel,
            baud_rate,
            timeout: Duration::from_secs(1),
        };
        stream.set_timeout(Duration::from_secs(1))?;

        if stream.probe() {
            remember_channel(&mac, channel);
            let _ = stream.clear(serialport::ClearBuffer::All);
            return Ok(stream);
        }

        last_error = io::Error::new(
            io::ErrorKind::TimedOut,
            format!("no ELM327 response on Bluetooth channel {channel}"),
        );
    }

    Err(last_error)
}

fn channels_for(mac: &str) -> Vec<u8> {
    let mut channels = Vec::new();

    if let Some(cached) = CHANNEL_CACHE
        .lock()
        .ok()
        .and_then(|cache| cache.as_ref().and_then(|cache| cache.get(mac).copied()))
    {
        channels.push(cached);
    }

    for channel in RFCOMM_CHANNELS {
        if !channels.contains(&channel) {
            channels.push(channel);
        }
    }

    channels
}

fn remember_channel(mac: &str, channel: u8) {
    if let Ok(mut cache) = CHANNEL_CACHE.lock() {
        cache
            .get_or_insert_with(HashMap::new)
            .insert(mac.to_string(), channel);
    }
}

fn mac_bytes(mac: &str) -> io::Result<[u8; 6]> {
    let mut bytes = [0u8; 6];
    let octets: Vec<&str> = mac.split(':').collect();
    if octets.len() != 6 {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "invalid MAC address"));
    }
    for (index, octet) in octets.iter().enumerate() {
        bytes[index] = u8::from_str_radix(octet, 16)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "invalid MAC address"))?;
    }
    Ok(bytes)
}

fn open_rfcomm(mac: &str, channel: u8) -> io::Result<RawFd> {
    let address = SockAddrRc {
        rc_family: AF_BLUETOOTH as u16,
        rc_bdaddr: mac_bytes(mac)?,
        rc_channel: channel,
    };

    let fd = unsafe { libc::socket(AF_BLUETOOTH, libc::SOCK_STREAM | libc::SOCK_CLOEXEC, BTPROTO_RFCOMM) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }

    // Non-blocking connect so a missing/unreachable adapter cannot stall the UI
    // for the kernel's default RFCOMM connect timeout.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        return Err(close_with_error(fd));
    }
    if unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
        return Err(close_with_error(fd));
    }

    let result = unsafe {
        libc::connect(
            fd,
            &address as *const SockAddrRc as *const libc::sockaddr,
            std::mem::size_of::<SockAddrRc>() as libc::socklen_t,
        )
    };

    if result != 0 {
        let error = io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::EINPROGRESS) {
            let _ = unsafe { libc::close(fd) };
            return Err(error);
        }

        let mut poll_fd = libc::pollfd {
            fd,
            events: libc::POLLOUT,
            revents: 0,
        };
        let ready = unsafe {
            libc::poll(
                &mut poll_fd,
                1,
                CONNECT_TIMEOUT.as_millis().min(i32::MAX as u128) as libc::c_int,
            )
        };

        if ready <= 0 {
            let _ = unsafe { libc::close(fd) };
            return Err(io::Error::new(io::ErrorKind::TimedOut, "Bluetooth connect timed out"));
        }

        let mut socket_error: libc::c_int = 0;
        let mut length = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
        let result = unsafe {
            libc::getsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_ERROR,
                &mut socket_error as *mut libc::c_int as *mut libc::c_void,
                &mut length,
            )
        };

        if result != 0 {
            return Err(close_with_error(fd));
        }
        if socket_error != 0 {
            let _ = unsafe { libc::close(fd) };
            return Err(io::Error::from_raw_os_error(socket_error));
        }
    }

    // Back to blocking mode: reads are bounded by SO_RCVTIMEO instead.
    if unsafe { libc::fcntl(fd, libc::F_SETFL, flags) } < 0 {
        return Err(close_with_error(fd));
    }

    Ok(fd)
}

fn close_with_error(fd: RawFd) -> io::Error {
    let error = io::Error::last_os_error();
    let _ = unsafe { libc::close(fd) };
    error
}

/// A serial-port-compatible view of an RFCOMM socket, so the rest of the OBD
/// stack (and `serialport::SerialPort`) works unchanged.
pub struct BluetoothSerialPort {
    fd: RawFd,
    mac: String,
    channel: u8,
    baud_rate: u32,
    timeout: Duration,
}

impl BluetoothSerialPort {
    /// Send `AT` and wait for the ELM327 prompt to validate the channel.
    fn probe(&mut self) -> bool {
        if self.write_all(b"AT\r").is_err() {
            return false;
        }

        let deadline = std::time::Instant::now() + PROBE_TIMEOUT;
        let mut byte = [0u8; 1];

        while std::time::Instant::now() < deadline {
            match self.read(&mut byte) {
                Ok(1) if byte[0] == b'>' => return true,
                Ok(1) => continue,
                Ok(_) => std::thread::sleep(Duration::from_millis(10)),
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(_) => return false,
            }
        }

        false
    }

    pub fn channel(&self) -> u8 {
        self.channel
    }
}

impl Drop for BluetoothSerialPort {
    fn drop(&mut self) {
        let _ = unsafe { libc::close(self.fd) };
    }
}

impl Read for BluetoothSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        loop {
            let read = unsafe { libc::recv(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len(), 0) };
            if read >= 0 {
                return Ok(read as usize);
            }

            let error = io::Error::last_os_error();
            match error.kind() {
                io::ErrorKind::Interrupted => continue,
                io::ErrorKind::WouldBlock => return Err(io::ErrorKind::TimedOut.into()),
                _ => return Err(error),
            }
        }
    }
}

impl Write for BluetoothSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        loop {
            // MSG_NOSIGNAL: a vanished adapter must not raise SIGPIPE.
            let written = unsafe {
                libc::send(
                    self.fd,
                    buf.as_ptr() as *const libc::c_void,
                    buf.len(),
                    libc::MSG_NOSIGNAL,
                )
            };
            if written >= 0 {
                return Ok(written as usize);
            }

            let error = io::Error::last_os_error();
            match error.kind() {
                io::ErrorKind::Interrupted => continue,
                io::ErrorKind::WouldBlock => return Err(io::ErrorKind::TimedOut.into()),
                _ => return Err(error),
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl SerialPort for BluetoothSerialPort {
    fn name(&self) -> Option<String> {
        Some(format!("{PORT_PREFIX}{}", self.mac))
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(self.baud_rate)
    }

    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        Ok(serialport::DataBits::Eight)
    }

    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        Ok(serialport::FlowControl::None)
    }

    fn parity(&self) -> serialport::Result<serialport::Parity> {
        Ok(serialport::Parity::None)
    }

    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        Ok(serialport::StopBits::One)
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        self.timeout = timeout;

        let timeval = libc::timeval {
            tv_sec: timeout.as_secs() as libc::time_t,
            tv_usec: timeout.subsec_micros() as libc::suseconds_t,
        };

        for option in [libc::SO_RCVTIMEO, libc::SO_SNDTIMEO] {
            let result = unsafe {
                libc::setsockopt(
                    self.fd,
                    libc::SOL_SOCKET,
                    option,
                    &timeval as *const libc::timeval as *const libc::c_void,
                    std::mem::size_of::<libc::timeval>() as libc::socklen_t,
                )
            };

            if result != 0 {
                return Err(serialport::Error::new(
                    serialport::ErrorKind::Io(io::ErrorKind::Other),
                    "failed to set Bluetooth socket timeout",
                ));
            }
        }

        Ok(())
    }

    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }

    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        Ok(())
    }

    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }

    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(true)
    }

    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }

    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        let mut available: libc::c_int = 0;
        let result = unsafe { libc::ioctl(self.fd, libc::FIONREAD, &mut available) };
        if result != 0 {
            return Ok(0);
        }
        Ok(available.max(0) as u32)
    }

    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }

    fn clear(&self, buffer: serialport::ClearBuffer) -> serialport::Result<()> {
        if matches!(
            buffer,
            serialport::ClearBuffer::Input | serialport::ClearBuffer::All
        ) {
            let mut scratch = [0u8; 256];
            loop {
                let read = unsafe {
                    libc::recv(
                        self.fd,
                        scratch.as_mut_ptr() as *mut libc::c_void,
                        scratch.len(),
                        libc::MSG_DONTWAIT,
                    )
                };
                if read <= 0 {
                    break;
                }
            }
        }

        Ok(())
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        let fd = unsafe { libc::dup(self.fd) };
        if fd < 0 {
            return Err(serialport::Error::new(
                serialport::ErrorKind::Io(io::ErrorKind::Other),
                "failed to duplicate Bluetooth socket",
            ));
        }

        Ok(Box::new(BluetoothSerialPort {
            fd,
            mac: self.mac.clone(),
            channel: self.channel,
            baud_rate: self.baud_rate,
            timeout: self.timeout,
        }))
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> serialport::Result<()> {
        // RFCOMM has no line coding: the value is informational only.
        self.baud_rate = baud_rate;
        Ok(())
    }

    fn set_data_bits(&mut self, _data_bits: serialport::DataBits) -> serialport::Result<()> {
        Ok(())
    }

    fn set_flow_control(&mut self, _flow_control: serialport::FlowControl) -> serialport::Result<()> {
        Ok(())
    }

    fn set_parity(&mut self, _parity: serialport::Parity) -> serialport::Result<()> {
        Ok(())
    }

    fn set_stop_bits(&mut self, _stop_bits: serialport::StopBits) -> serialport::Result<()> {
        Ok(())
    }

    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }

    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_port_names() {
        assert_eq!(
            parse_mac("BT:AA:BB:CC:DD:EE:FF").as_deref(),
            Some("AA:BB:CC:DD:EE:FF")
        );
        assert_eq!(
            parse_mac("BT:aa:bb:cc:dd:ee:ff").as_deref(),
            Some("AA:BB:CC:DD:EE:FF")
        );
        assert!(is_bluetooth_port("BT:AA:BB:CC:DD:EE:FF"));
        assert!(!is_bluetooth_port("/dev/ttyUSB0"));
        assert!(!is_bluetooth_port("BT:AA:BB:CC:DD:EE"));
        assert!(!is_bluetooth_port("BT:AA:BB:CC:DD:EE:ZZ"));
    }

    #[test]
    fn parses_bluetoothctl_device_list() {
        let output = "\
Device AA:BB:CC:DD:EE:FF OBDII
Device 11:22:33:44:55:66 Headset With Spaces
garbage line
";
        let devices = parse_device_list(output);
        assert_eq!(
            devices,
            vec![
                ("AA:BB:CC:DD:EE:FF".to_string(), "OBDII".to_string()),
                (
                    "11:22:33:44:55:66".to_string(),
                    "Headset With Spaces".to_string()
                ),
            ]
        );
    }

    #[test]
    fn detects_spp_service() {
        let info = "\
Device AA:BB:CC:DD:EE:FF (public)
\tName: OBDII
\tUUID: Serial Port     (00001101-0000-1000-8000-00805f9b34fb)
";
        assert!(has_spp_service(info));

        let ble = "\
Device AA:BB:CC:DD:EE:FF (public)
\tName: OBDII
\tUUID: Vendor specific (0000ffe0-0000-1000-8000-00805f9b34fb)
";
        assert!(!has_spp_service(ble));
    }
}
