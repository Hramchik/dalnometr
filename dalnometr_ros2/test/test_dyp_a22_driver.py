"""Unit tests for the DYP-A22 I2C driver (no hardware required)."""

import sys
import types
import pytest

# ---------------------------------------------------------------------------
# Stub smbus2 so the driver can be imported without the real package
# ---------------------------------------------------------------------------
smbus2_stub = types.ModuleType("smbus2")


class _FakeSMBus:
    def __init__(self, bus_id=1):
        self.bus_id = bus_id
        self._written = []
        self._read_data = []      # pre-load bytes to return from read_i2c_block_data

    def write_byte(self, addr, byte):
        self._written.append(("write_byte", addr, byte))

    def write_i2c_block_data(self, addr, reg, data):
        self._written.append(("write_block", addr, reg, list(data)))

    def read_i2c_block_data(self, addr, reg, length):
        return self._read_data[:length]

    def close(self):
        pass


smbus2_stub.SMBus = _FakeSMBus
sys.modules["smbus2"] = smbus2_stub

from dalnometr_ros2.dyp_a22_driver import DypA22, DypA22Error, scan_bus, DEFAULT_ADDRESS  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_payload(distance_mm: int):
    high = (distance_mm >> 8) & 0xFF
    low = distance_mm & 0xFF
    checksum = (high + low) & 0xFF
    return [high, low, checksum]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestReadDistance:
    def test_valid_reading(self):
        bus = _FakeSMBus()
        bus._read_data = _make_payload(1200)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        # Patch sleep so tests run fast
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        assert sensor.read_distance_mm() == 1200.0

    def test_checksum_mismatch_raises(self):
        bus = _FakeSMBus()
        bus._read_data = [0x04, 0xB0, 0x00]  # bad checksum
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        with pytest.raises(DypA22Error, match="Checksum"):
            sensor.read_distance_mm()

    def test_zero_distance(self):
        bus = _FakeSMBus()
        bus._read_data = _make_payload(0)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        assert sensor.read_distance_mm() == 0.0

    def test_max_distance(self):
        bus = _FakeSMBus()
        bus._read_data = _make_payload(4500)
        sensor = DypA22(bus, DEFAULT_ADDRESS)
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        assert sensor.read_distance_mm() == 4500.0


class TestChangeAddress:
    def test_valid_address_change(self):
        bus = _FakeSMBus()
        bus._read_data = _make_payload(500)
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        sensor = DypA22(bus, 0x57)
        sensor.change_address(0x58)
        assert sensor.address == 0x58
        # Verify the magic bytes were written
        last_write = bus._written[-1]
        assert last_write[0] == "write_block"
        assert last_write[3][-1] == 0x58

    def test_invalid_address_raises(self):
        bus = _FakeSMBus()
        sensor = DypA22(bus, 0x57)
        with pytest.raises(ValueError):
            sensor.change_address(0x78)   # out of range

    def test_address_zero_raises(self):
        bus = _FakeSMBus()
        sensor = DypA22(bus, 0x57)
        with pytest.raises(ValueError):
            sensor.change_address(0x00)


class TestScanBus:
    def test_finds_default_sensor(self):
        bus = _FakeSMBus()
        bus._read_data = _make_payload(300)
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        found = scan_bus(bus)
        assert DEFAULT_ADDRESS in found

    def test_empty_when_no_response(self):
        bus = _FakeSMBus()
        bus._read_data = [0x00, 0x00, 0xFF]  # bad checksum → ping fails
        import dalnometr_ros2.dyp_a22_driver as drv
        drv._MEASURE_DELAY = 0
        found = scan_bus(bus)
        assert found == []
