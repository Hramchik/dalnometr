"""Unit tests for the DYP-A22 I2C driver (no hardware required)."""

import math
import sys
import types
import pytest

# ---------------------------------------------------------------------------
# Stub smbus2: emulates i2c_rdwr / i2c_msg without real hardware
# ---------------------------------------------------------------------------

class _I2cMsg:
    """Minimal stand-in for smbus2.i2c_msg."""

    def __init__(self, data: list):
        self._data = list(data)
        self._type = None   # 'write' or 'read'
        self._addr = 0

    def __iter__(self):
        return iter(self._data)

    def __bytes__(self):
        return bytes(self._data)

    @staticmethod
    def write(addr: int, data: list) -> "_I2cMsg":
        msg = _I2cMsg(data)
        msg._type = "write"
        msg._addr = addr
        return msg

    @staticmethod
    def read(addr: int, length: int) -> "_I2cMsg":
        msg = _I2cMsg([0] * length)
        msg._type = "read"
        msg._addr = addr
        return msg


class _FakeSMBus:
    def __init__(self, bus_id: int = 1):
        self.bus_id = bus_id
        self.written: list = []     # list of (addr, [bytes]) for each write msg
        self._read_data: list = []  # bytes returned for the next read
        self.raise_oserror: bool = False  # simulate missing device

    def i2c_rdwr(self, *msgs):
        if self.raise_oserror:
            raise OSError(121, "Remote I/O error")
        for msg in msgs:
            if msg._type == "write":
                self.written.append((msg._addr, list(msg._data)))
            elif msg._type == "read":
                msg._data = list(self._read_data[: len(msg._data)])

    def close(self):
        pass


smbus2_stub = types.ModuleType("smbus2")
smbus2_stub.SMBus = _FakeSMBus
smbus2_stub.i2c_msg = _I2cMsg
sys.modules["smbus2"] = smbus2_stub


from dalnometr_ros2.dyp_a22_driver import (  # noqa: E402
    DypA22,
    DypA22Error,
    scan_bus,
    DEFAULT_ADDRESS,
    _FRAME_HEADER,
    _NO_ECHO,
)

import dalnometr_ros2.dyp_a22_driver as _drv  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers — 4-byte frame: [header, dist_high, dist_low, checksum]
# ---------------------------------------------------------------------------

def _frame(distance_mm: int) -> list:
    """Build a valid 4-byte sensor frame for a given distance."""
    high = (distance_mm >> 8) & 0xFF
    low = distance_mm & 0xFF
    return [_FRAME_HEADER, high, low, (high + low) & 0xFF]


def _no_echo_frame() -> list:
    """4-byte frame returned when no echo is received."""
    return [_FRAME_HEADER, 0xFF, 0xFF, 0x09]


# Speed up all tests — no real delays needed.
_drv._MEASURE_DELAY = 0


# ---------------------------------------------------------------------------
# TestInit
# ---------------------------------------------------------------------------

class TestInit:
    def test_returns_true_when_sensor_responds(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(500)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.init() is True

    def test_returns_false_when_no_device(self):
        bus = _FakeSMBus()
        bus.raise_oserror = True
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.init() is False


# ---------------------------------------------------------------------------
# TestPing
# ---------------------------------------------------------------------------

class TestPing:
    def test_returns_true_on_ack(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(300)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.ping() is True

    def test_returns_true_on_no_echo_frame(self):
        # A no-echo response still means the device is present
        bus = _FakeSMBus()
        bus._read_data = _no_echo_frame()
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.ping() is True

    def test_returns_false_on_oserror(self):
        bus = _FakeSMBus()
        bus.raise_oserror = True
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.ping() is False


# ---------------------------------------------------------------------------
# TestReadDistance
# ---------------------------------------------------------------------------

class TestReadDistance:
    def test_valid_reading(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(1200)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.read_distance_mm() == 1200.0

    def test_zero_distance(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(0)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.read_distance_mm() == 0.0

    def test_max_distance(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(4500)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.read_distance_mm() == 4500.0

    def test_no_echo_returns_inf(self):
        bus = _FakeSMBus()
        bus._read_data = _no_echo_frame()
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert math.isinf(sensor.read_distance_mm())

    def test_i2c_error_raises_dyp_error(self):
        bus = _FakeSMBus()
        bus.raise_oserror = True
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        with pytest.raises(DypA22Error):
            sensor.read_distance_mm()


# ---------------------------------------------------------------------------
# TestReadDistanceStrict
# ---------------------------------------------------------------------------

class TestReadDistanceStrict:
    def test_valid_reading(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(600)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert sensor.read_distance_mm_strict() == 600.0

    def test_no_echo_returns_inf(self):
        bus = _FakeSMBus()
        bus._read_data = _no_echo_frame()
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        assert math.isinf(sensor.read_distance_mm_strict())

    def test_bad_header_raises(self):
        bus = _FakeSMBus()
        bad_frame = [0x99, 0x01, 0xF4, 0xF5]  # header != 0x02
        bus._read_data = bad_frame
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        with pytest.raises(DypA22Error, match="[Hh]eader"):
            sensor.read_distance_mm_strict()


# ---------------------------------------------------------------------------
# TestChangeAddress
# ---------------------------------------------------------------------------

class TestChangeAddress:
    def test_valid_address_change(self):
        bus = _FakeSMBus()
        sensor = DypA22(bus, 0x57)
        sensor.change_address(0x58)
        assert sensor.address == 0x58
        # Verify the magic + new address was written
        last_addr, last_data = bus.written[-1]
        assert last_addr == 0x57
        assert last_data == [0x55, 0xAA, 0xA2, 0x58]

    def test_invalid_address_too_high_raises(self):
        bus = _FakeSMBus()
        sensor = DypA22(bus, 0x57)
        with pytest.raises(ValueError):
            sensor.change_address(0x78)

    def test_invalid_address_zero_raises(self):
        bus = _FakeSMBus()
        sensor = DypA22(bus, 0x57)
        with pytest.raises(ValueError):
            sensor.change_address(0x00)

    def test_i2c_error_during_change_raises_dyp_error(self):
        bus = _FakeSMBus()
        bus.raise_oserror = True
        sensor = DypA22(bus, 0x57)
        with pytest.raises(DypA22Error):
            sensor.change_address(0x58)


# ---------------------------------------------------------------------------
# TestScanBus
# ---------------------------------------------------------------------------

class TestScanBus:
    def test_finds_default_sensor(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(300)
        found = scan_bus(bus)
        assert DEFAULT_ADDRESS in found

    def test_finds_sensor_on_no_echo(self):
        # Device is present even when returning no-echo
        bus = _FakeSMBus()
        bus._read_data = _no_echo_frame()
        found = scan_bus(bus)
        assert DEFAULT_ADDRESS in found

    def test_empty_when_no_device(self):
        bus = _FakeSMBus()
        bus.raise_oserror = True
        found = scan_bus(bus)
        assert found == []

    def test_finds_multiple_sensors(self):
        bus = _FakeSMBus()
        bus._read_data = _frame(200)
        found = scan_bus(bus, candidates=[0x57, 0x58, 0x74])
        assert 0x57 in found
        assert 0x58 in found
        assert 0x74 in found
