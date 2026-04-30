"""
DYP-A22 (AA2211AC) ultrasonic sensor I2C driver.

Protocol (I2C) — 4-byte frame:
  Default address : 0x74  (factory default for this unit)
  Start measure   : write single byte 0x01
  Read result     : read 4 bytes -> [header, high, low, checksum]
    header   : 0x02 (fixed frame marker)
    Distance : (high << 8) | low  in mm
    No-echo  : high == 0xFF and low == 0xFF  (0xFFFF sentinel)

Address change:
  Write 4 bytes: [0x55, 0xAA, 0xA2, new_addr]
  The sensor resets and comes up on the new address.
  Valid range: 0x08 – 0x77 (standard 7-bit I2C range).

Note: raw i2c_rdwr is used for reads to avoid the spurious register-address
byte that read_i2c_block_data sends before switching to read mode.
"""

import math
import time
import smbus2


DEFAULT_ADDRESS = 0x74

_CMD_MEASURE = 0x01
_FRAME_HEADER = 0x02
_NO_ECHO = 0xFFFF          # sensor sentinel: no echo received
_ADDR_CHANGE_MAGIC = bytes([0x55, 0xAA, 0xA2])

# Seconds to wait for conversion after triggering a measurement.
# AA2211AC datasheet: ~120-200 ms typical. 200 ms leaves margin without
# exceeding the 4 Hz (250 ms) timer period at default publish rate.
# If readings freeze, lower publish_rate_hz below 1 / _MEASURE_DELAY.
_MEASURE_DELAY = 0.200


class DypA22Error(Exception):
    pass


class DypA22:
    def __init__(self, bus: smbus2.SMBus, address: int = DEFAULT_ADDRESS):
        self._bus = bus
        self._address = address

    @property
    def address(self) -> int:
        return self._address

    def init(self) -> bool:
        """
        Verify the sensor is present and returns a plausible reading.
        Returns True on success, False if the device is absent or unresponsive.
        Intended for startup checks — does not raise exceptions.
        """
        if not self.ping():
            return False
        try:
            self.read_distance_mm()
            return True
        except DypA22Error:
            return False

    def ping(self) -> bool:
        """
        Return True if the sensor responds on its I2C address.
        A no-echo (0xFFFF) response still counts as a live device.
        """
        try:
            write_msg = smbus2.i2c_msg.write(self._address, [_CMD_MEASURE])
            self._bus.i2c_rdwr(write_msg)
            time.sleep(_MEASURE_DELAY)
            read_msg = smbus2.i2c_msg.read(self._address, 4)
            self._bus.i2c_rdwr(read_msg)
            return True  # device ACKed — it's there
        except OSError:
            return False

    def read_distance_mm(self) -> float:
        """
        Trigger a measurement and return distance in millimetres.
        Returns math.inf when the sensor reports no echo (0xFFFF sentinel).
        Raises DypA22Error on I2C communication error.
        """
        header, high, low = self._read_frame()
        raw = (high << 8) | low
        if raw == _NO_ECHO:
            return math.inf
        return float(raw)

    def read_distance_mm_strict(self) -> float:
        """
        Same as read_distance_mm but raises DypA22Error on wrong frame header.
        """
        header, high, low = self._read_frame()
        if header != _FRAME_HEADER:
            raise DypA22Error(
                f"Unexpected frame header 0x{header:02X} at 0x{self._address:02X}"
            )
        raw = (high << 8) | low
        if raw == _NO_ECHO:
            return math.inf
        return float(raw)

    def read_raw_bytes(self) -> list:
        """Return raw 4 bytes from sensor without validation (useful for debugging)."""
        return list(self._read_raw())

    def change_address(self, new_address: int) -> None:
        """
        Permanently change the sensor I2C address.

        new_address must be in [0x08, 0x77].
        After the call the driver's internal address is updated so that
        subsequent reads use the new address immediately.
        """
        if not (0x08 <= new_address <= 0x77):
            raise ValueError(
                f"I2C address 0x{new_address:02X} is out of valid range [0x08, 0x77]"
            )
        payload = list(_ADDR_CHANGE_MAGIC) + [new_address]
        try:
            write_msg = smbus2.i2c_msg.write(self._address, payload)
            self._bus.i2c_rdwr(write_msg)
        except OSError as exc:
            raise DypA22Error(
                f"Failed to change address from 0x{self._address:02X}: {exc}"
            ) from exc
        # Give sensor time to reboot on the new address
        time.sleep(0.200)
        self._address = new_address

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_raw(self) -> bytes:
        """Trigger measurement and return 4 raw bytes using i2c_rdwr."""
        write_msg = smbus2.i2c_msg.write(self._address, [_CMD_MEASURE])
        self._bus.i2c_rdwr(write_msg)
        time.sleep(_MEASURE_DELAY)
        read_msg = smbus2.i2c_msg.read(self._address, 4)
        self._bus.i2c_rdwr(read_msg)
        return bytes(read_msg)

    def _read_frame(self) -> tuple:
        """
        Return (header, high, low) from a fresh measurement.
        Raises DypA22Error on I2C error.
        """
        try:
            data = self._read_raw()
        except OSError as exc:
            raise DypA22Error(f"I2C error at 0x{self._address:02X}: {exc}") from exc
        # Frame: [header, dist_high, dist_low, checksum]
        return data[0], data[1], data[2]


def scan_bus(bus: smbus2.SMBus, candidates: list = None) -> list:
    """
    Return list of I2C addresses where DYP-A22 sensors respond.

    candidates: list of addresses to probe; defaults to the factory address only.
    """
    if candidates is None:
        candidates = [DEFAULT_ADDRESS]
    found = []
    for addr in candidates:
        sensor = DypA22(bus, addr)
        if sensor.ping():
            found.append(addr)
    return found
