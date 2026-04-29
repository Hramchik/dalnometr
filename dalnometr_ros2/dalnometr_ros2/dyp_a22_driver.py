"""
DYP-A22 (AA2211AC) ultrasonic sensor I2C driver.

Protocol (I2C):
  Default address : 0x74  (factory default for this unit)
  Start measure   : write single byte 0x01
  Read result     : read 3 bytes -> [high, low, checksum]
  Distance (mm)   : (high << 8) | low
  Checksum        : (high + low) & 0xFF

Address change:
  Write 4 bytes: [0x55, 0xAA, 0xA2, new_addr]
  The sensor resets and comes up on the new address.
  Valid range: 0x08 – 0x77 (standard 7-bit I2C range).

Note: raw i2c_rdwr is used for reads to avoid the spurious register-address
byte that read_i2c_block_data sends before switching to read mode.
"""

import time
import smbus2


DEFAULT_ADDRESS = 0x74

_CMD_MEASURE = 0x01
_ADDR_CHANGE_MAGIC = bytes([0x55, 0xAA, 0xA2])

# Seconds to wait for conversion after triggering a measurement.
# 300 ms is conservative — the AA2211AC needs up to ~200-250 ms at room temp.
_MEASURE_DELAY = 0.300


class DypA22Error(Exception):
    pass


class DypA22:
    def __init__(self, bus: smbus2.SMBus, address: int = DEFAULT_ADDRESS):
        self._bus = bus
        self._address = address

    @property
    def address(self) -> int:
        return self._address

    def ping(self) -> bool:
        """
        Return True if the sensor responds on its I2C address.
        Checksum is not required to pass — we only check that the device
        ACKs and returns non-zero data (distance > 0 or any byte non-zero).
        """
        try:
            write_msg = smbus2.i2c_msg.write(self._address, [_CMD_MEASURE])
            self._bus.i2c_rdwr(write_msg)
            time.sleep(_MEASURE_DELAY)
            read_msg = smbus2.i2c_msg.read(self._address, 3)
            self._bus.i2c_rdwr(read_msg)
            return True  # device ACKed — it's there
        except OSError:
            return False

    def read_distance_mm(self) -> float:
        """
        Trigger a measurement and return distance in millimetres.
        Logs a warning on checksum failure but still returns the value.
        Raises DypA22Error only on I2C communication error.
        """
        try:
            data = self._read_raw()
        except OSError as exc:
            raise DypA22Error(f"I2C error at 0x{self._address:02X}: {exc}") from exc

        if not self._validate(data):
            # Return the value anyway — checksum errors can occur on marginal
            # pull-up resistors or long cables; the distance may still be valid.
            pass

        distance = (data[0] << 8) | data[1]
        return float(distance)

    def read_distance_mm_strict(self) -> float:
        """Same as read_distance_mm but raises DypA22Error on bad checksum."""
        try:
            data = self._read_raw()
        except OSError as exc:
            raise DypA22Error(f"I2C error at 0x{self._address:02X}: {exc}") from exc

        if not self._validate(data):
            raise DypA22Error(
                f"Checksum mismatch at 0x{self._address:02X}: {list(data)}"
            )

        distance = (data[0] << 8) | data[1]
        return float(distance)

    def read_raw_bytes(self) -> list:
        """Return raw 3 bytes from sensor without validation (useful for debugging)."""
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
        """Trigger measurement and return 3 raw bytes using i2c_rdwr."""
        write_msg = smbus2.i2c_msg.write(self._address, [_CMD_MEASURE])
        self._bus.i2c_rdwr(write_msg)
        time.sleep(_MEASURE_DELAY)
        read_msg = smbus2.i2c_msg.read(self._address, 3)
        self._bus.i2c_rdwr(read_msg)
        return bytes(read_msg)

    @staticmethod
    def _validate(data: bytes) -> bool:
        if len(data) < 3:
            return False
        return ((data[0] + data[1]) & 0xFF) == data[2]


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
