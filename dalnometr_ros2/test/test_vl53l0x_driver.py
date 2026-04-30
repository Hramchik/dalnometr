"""Unit tests for the VL53L0X driver (no hardware required)."""

import math
import sys
import types
import pytest

# ---------------------------------------------------------------------------
# Stub VL53L0X library so driver can be imported without real hardware
# ---------------------------------------------------------------------------

class _FakeTof:
    def __init__(self, *, i2c_bus=1, i2c_address=0x29):
        self.i2c_address = i2c_address
        self._distance = 500       # mm returned by get_distance()
        self._raise_on_open = False
        self._raise_on_read = False

    def open(self):
        if self._raise_on_open:
            raise OSError("Device not found")

    def start_ranging(self, mode):
        pass

    def get_distance(self):
        if self._raise_on_read:
            raise OSError("Read error")
        return self._distance

    def stop_ranging(self):
        pass

    def close(self):
        pass

    def change_address(self, new_addr):
        self.i2c_address = new_addr


class _FakeAccuracyMode:
    BETTER = 2


_last_tof_instance = None  # lets tests inspect the created object


def _fake_vl53l0x_ctor(*, i2c_bus=1, i2c_address=0x29):
    global _last_tof_instance
    _last_tof_instance = _FakeTof(i2c_bus=i2c_bus, i2c_address=i2c_address)
    return _last_tof_instance


vl53l0x_stub = types.ModuleType("VL53L0X")
vl53l0x_stub.VL53L0X = _fake_vl53l0x_ctor
vl53l0x_stub.Vl53l0xAccuracyMode = _FakeAccuracyMode
sys.modules["VL53L0X"] = vl53l0x_stub


from dalnometr_ros2.vl53l0x_driver import (  # noqa: E402
    Vl53l0x,
    Vl53l0xError,
    scan_bus,
    DEFAULT_ADDRESS,
    _NO_ECHO,
)


# ---------------------------------------------------------------------------
# TestInit
# ---------------------------------------------------------------------------

class TestInit:
    def test_returns_true_on_success(self):
        sensor = Vl53l0x(bus_id=1, address=DEFAULT_ADDRESS)
        assert sensor.init() is True

    def test_returns_false_when_device_missing(self):
        sensor = Vl53l0x(bus_id=1, address=DEFAULT_ADDRESS)
        _last_tof_instance  # ensure ctor ran once
        # Patch ctor to raise on open
        import dalnometr_ros2.vl53l0x_driver as drv
        original = drv._lib.VL53L0X

        def failing_ctor(**kw):
            t = _FakeTof(**kw)
            t._raise_on_open = True
            return t

        drv._lib.VL53L0X = failing_ctor
        try:
            assert sensor.init() is False
        finally:
            drv._lib.VL53L0X = original

    def test_close_before_init_is_safe(self):
        sensor = Vl53l0x(bus_id=1, address=DEFAULT_ADDRESS)
        sensor.close()  # should not raise


# ---------------------------------------------------------------------------
# TestReadDistance
# ---------------------------------------------------------------------------

class TestReadDistance:
    def _sensor(self, distance_mm: int) -> Vl53l0x:
        sensor = Vl53l0x(bus_id=1, address=DEFAULT_ADDRESS)
        sensor.init()
        _last_tof_instance._distance = distance_mm
        return sensor

    def test_valid_reading(self):
        sensor = self._sensor(800)
        assert sensor.read_distance_mm() == 800.0

    def test_min_valid_reading(self):
        sensor = self._sensor(30)
        assert sensor.read_distance_mm() == 30.0

    def test_no_echo_returns_inf(self):
        sensor = self._sensor(_NO_ECHO)
        assert math.isinf(sensor.read_distance_mm())

    def test_zero_distance_returns_inf(self):
        sensor = self._sensor(0)
        assert math.isinf(sensor.read_distance_mm())

    def test_raises_when_not_initialized(self):
        sensor = Vl53l0x(bus_id=1, address=DEFAULT_ADDRESS)
        with pytest.raises(Vl53l0xError, match="[Nn]ot initialized|init"):
            sensor.read_distance_mm()

    def test_raises_on_read_error(self):
        sensor = self._sensor(500)
        _last_tof_instance._raise_on_read = True
        with pytest.raises(Vl53l0xError):
            sensor.read_distance_mm()


# ---------------------------------------------------------------------------
# TestChangeAddress
# ---------------------------------------------------------------------------

class TestChangeAddress:
    def test_valid_address_change(self):
        sensor = Vl53l0x(bus_id=1, address=0x29)
        sensor.init()
        sensor.change_address(0x30)
        assert sensor.address == 0x30

    def test_invalid_address_raises(self):
        sensor = Vl53l0x(bus_id=1, address=0x29)
        sensor.init()
        with pytest.raises(ValueError):
            sensor.change_address(0x78)

    def test_not_initialized_raises(self):
        sensor = Vl53l0x(bus_id=1, address=0x29)
        with pytest.raises(Vl53l0xError):
            sensor.change_address(0x30)


# ---------------------------------------------------------------------------
# TestScanBus
# ---------------------------------------------------------------------------

class TestScanBus:
    def test_finds_default_sensor(self):
        found = scan_bus(bus_id=1)
        assert DEFAULT_ADDRESS in found

    def test_finds_multiple_sensors(self):
        found = scan_bus(bus_id=1, candidates=[0x29, 0x30])
        assert 0x29 in found
        assert 0x30 in found

    def test_empty_when_init_fails(self):
        import dalnometr_ros2.vl53l0x_driver as drv
        original = drv._lib.VL53L0X

        def failing_ctor(**kw):
            t = _FakeTof(**kw)
            t._raise_on_open = True
            return t

        drv._lib.VL53L0X = failing_ctor
        try:
            found = scan_bus(bus_id=1, candidates=[0x29])
            assert found == []
        finally:
            drv._lib.VL53L0X = original
