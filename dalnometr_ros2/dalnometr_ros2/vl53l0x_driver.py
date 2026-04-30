"""
VL53L0X Time-of-Flight laser distance sensor I2C driver.

Uses the VL53L0X Python library (pip install VL53L0X) which wraps
ST's official API via smbus.

Protocol summary:
  Default I2C address : 0x29
  Range               : 30 – 2000 mm (reliable up to ~1200 mm outdoors)
  No-echo sentinel    : 8190 (0x1FFE) — returned when no target in range

Address change (multiple sensors):
  VL53L0X changes address via I2C command, but all units start at 0x29.
  To use multiple sensors simultaneously you must control each sensor's
  XSHUT pin (pull LOW to disable, HIGH to enable), then change the address
  of the active sensor before enabling the next one.
"""

import math
import VL53L0X as _lib

DEFAULT_ADDRESS = 0x29

# Measurement returned by the library when no target is detected.
_NO_ECHO = 8190

# Accuracy mode used for ranging. BETTER balances speed and accuracy.
_ACCURACY_MODE = _lib.Vl53l0xAccuracyMode.BETTER


class Vl53l0xError(Exception):
    pass


class Vl53l0x:
    def __init__(self, bus_id: int, address: int = DEFAULT_ADDRESS):
        self._bus_id = bus_id
        self._address = address
        self._tof = None

    @property
    def address(self) -> int:
        return self._address

    def init(self) -> bool:
        """
        Open the sensor, run the initialization sequence, and start ranging.
        Returns True on success, False if the device is absent or fails to init.
        Safe to call multiple times — closes the previous session first.
        """
        self.close()
        try:
            tof = _lib.VL53L0X(i2c_bus=self._bus_id, i2c_address=self._address)
            tof.open()
            tof.start_ranging(_ACCURACY_MODE)
            self._tof = tof
            return True
        except Exception:
            return False

    def read_distance_mm(self) -> float:
        """
        Return the latest distance measurement in millimetres.
        Returns math.inf when no target is detected (no-echo sentinel).
        Raises Vl53l0xError if the sensor has not been initialized or on error.
        """
        if self._tof is None:
            raise Vl53l0xError(
                f"Sensor at 0x{self._address:02X} is not initialized — call init() first"
            )
        try:
            dist = self._tof.get_distance()
        except Exception as exc:
            raise Vl53l0xError(
                f"Read error at 0x{self._address:02X}: {exc}"
            ) from exc

        if dist <= 0 or dist >= _NO_ECHO:
            return math.inf
        return float(dist)

    def change_address(self, new_address: int) -> None:
        """
        Change the sensor's I2C address in firmware.
        new_address must be in [0x08, 0x77].

        Note: all VL53L0X sensors boot at 0x29.  When multiple sensors share
        a bus you must use XSHUT pins to isolate each sensor before calling
        this method so that only one unit receives the command.

        After the call the driver reconnects on the new address automatically.
        """
        if not (0x08 <= new_address <= 0x77):
            raise ValueError(
                f"I2C address 0x{new_address:02X} is out of valid range [0x08, 0x77]"
            )
        if self._tof is None:
            raise Vl53l0xError(
                "Sensor must be initialized before changing address — call init() first"
            )
        try:
            self._tof.change_address(new_address)
        except Exception as exc:
            raise Vl53l0xError(
                f"Failed to change address from 0x{self._address:02X}: {exc}"
            ) from exc
        self._address = new_address
        # Re-initialize on the new address so ranging continues uninterrupted.
        if not self.init():
            raise Vl53l0xError(
                f"Sensor did not respond after address change to 0x{new_address:02X}"
            )

    def close(self) -> None:
        """Stop ranging and release the I2C handle."""
        if self._tof is not None:
            try:
                self._tof.stop_ranging()
                self._tof.close()
            except Exception:
                pass
            self._tof = None


def scan_bus(bus_id: int, candidates: list = None) -> list:
    """
    Return list of I2C addresses where VL53L0X sensors respond.

    Probes each candidate address by running a full init() and immediately
    closing the session.  candidates defaults to [DEFAULT_ADDRESS].
    """
    if candidates is None:
        candidates = [DEFAULT_ADDRESS]
    found = []
    for addr in candidates:
        sensor = Vl53l0x(bus_id, addr)
        if sensor.init():
            sensor.close()
            found.append(addr)
    return found
