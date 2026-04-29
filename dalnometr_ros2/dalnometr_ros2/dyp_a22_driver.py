"""
DYP-A22 (AA2211AC) ultrasonic sensor I2C driver.

Protocol (I2C):
  Default address : 0x57
  Start measure   : write single byte 0x01
  Read result     : read 3 bytes -> [high, low, checksum]
  Distance (mm)   : (high << 8) | low
  Checksum        : (high + low) & 0xFF

Address change:
  Write 4 bytes: [0x55, 0xAA, 0xA2, new_addr]
  The sensor resets and comes up on the new address.
  Valid range: 0x08 – 0x77 (standard 7-bit I2C range).
"""

import time
import smbus2


DEFAULT_ADDRESS = 0x57

_CMD_MEASURE = 0x01
_ADDR_CHANGE_MAGIC = bytes([0x55, 0xAA, 0xA2])

# Seconds to wait for conversion after triggering a measurement
_MEASURE_DELAY = 0.060


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
        """Return True if the sensor responds on its I2C address."""
        try:
            self._bus.write_byte(self._address, _CMD_MEASURE)
            time.sleep(_MEASURE_DELAY)
            data = self._bus.read_i2c_block_data(self._address, 0, 3)
            return self._validate(data)
        except OSError:
            return False

    def read_distance_mm(self) -> float:
        """
        Trigger a measurement and return distance in millimetres.
        Raises DypA22Error on checksum failure or I2C error.
        """
        try:
            self._bus.write_byte(self._address, _CMD_MEASURE)
            time.sleep(_MEASURE_DELAY)
            data = self._bus.read_i2c_block_data(self._address, 0, 3)
        except OSError as exc:
            raise DypA22Error(f"I2C error at 0x{self._address:02X}: {exc}") from exc

        if not self._validate(data):
            raise DypA22Error(
                f"Checksum mismatch at 0x{self._address:02X}: {data}"
            )

        distance = (data[0] << 8) | data[1]
        return float(distance)

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
            self._bus.write_i2c_block_data(self._address, 0, payload)
        except OSError as exc:
            raise DypA22Error(
                f"Failed to change address from 0x{self._address:02X}: {exc}"
            ) from exc
        # Give sensor time to reboot on the new address
        time.sleep(0.200)
        self._address = new_address

    @staticmethod
    def _validate(data: list) -> bool:
        if len(data) < 3:
            return False
        return ((data[0] + data[1]) & 0xFF) == data[2]


def scan_bus(bus: smbus2.SMBus, candidates: list[int] | None = None) -> list[int]:
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
