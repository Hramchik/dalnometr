"""
ROS2 Humble node for VL53L0X Time-of-Flight sensors on I2C.

Published topics (one per sensor):
  ~/sensor_<hex_addr>   sensor_msgs/Range

Services:
  ~/change_address      dalnometr_msgs/srv/ChangeAddress

Parameters:
  i2c_bus          (int,   default 1)       — Linux I2C bus index (/dev/i2c-N)
  addresses        (int[], default [41])    — addresses to probe on startup (decimal; 41 = 0x29)
  publish_rate_hz  (float, default 10.0)   — measurement frequency per sensor
  frame_prefix     (str,   default "tof")  — tf frame_id prefix
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from dalnometr_msgs.srv import ChangeAddress
from dalnometr_ros2.vl53l0x_driver import Vl53l0x, Vl53l0xError, scan_bus

# VL53L0X physical beam characteristics
_FIELD_OF_VIEW_RAD = 0.4363  # ~25 degrees
_MIN_RANGE_M = 0.030
_MAX_RANGE_M = 2.000


class Vl53l0xNode(Node):
    def __init__(self):
        super().__init__("vl53l0x_node")

        self._declare_parameters()

        bus_id: int = self.get_parameter("i2c_bus").value
        addresses: list = list(self.get_parameter("addresses").value)
        rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._frame_prefix: str = self.get_parameter("frame_prefix").value
        self._bus_id = bus_id

        self.get_logger().info(
            "Scanning /dev/i2c-{} at: {}".format(
                bus_id, ", ".join(f"0x{a:02X}" for a in addresses)
            )
        )

        found = scan_bus(bus_id, addresses)
        if not found:
            self.get_logger().warn("No VL53L0X sensors detected on I2C bus.")

        self._sensors: dict = {}
        self._pubs: dict = {}

        for addr in found:
            self._register_sensor(addr)

        self._change_addr_srv = self.create_service(
            ChangeAddress, "~/change_address", self._handle_change_address
        )

        self._timer = self.create_timer(1.0 / rate_hz, self._publish_all)

        self.get_logger().info(
            "Node ready — {} sensor(s), {:.1f} Hz".format(
                len(self._sensors), rate_hz
            )
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("addresses", [0x29])  # 41 decimal
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_prefix", "tof")

    # ------------------------------------------------------------------
    # Sensor registration
    # ------------------------------------------------------------------

    def _register_sensor(self, address: int) -> None:
        sensor = Vl53l0x(self._bus_id, address)
        if not sensor.init():
            self.get_logger().error(
                "Failed to initialize VL53L0X at 0x{:02X}".format(address)
            )
            return
        topic = "sensor_{:02x}".format(address)
        pub = self.create_publisher(Range, "~/{}".format(topic), 10)
        self._sensors[address] = sensor
        self._pubs[address] = pub
        self.get_logger().info(
            "Registered sensor 0x{:02X} → topic ~/{}".format(address, topic)
        )

    # ------------------------------------------------------------------
    # Timer callback — read all sensors and publish
    # ------------------------------------------------------------------

    def _publish_all(self) -> None:
        now = self.get_clock().now().to_msg()
        for addr, sensor in list(self._sensors.items()):
            try:
                distance_mm = sensor.read_distance_mm()
            except Vl53l0xError as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
                continue

            msg = Range()
            msg.header = Header()
            msg.header.stamp = now
            msg.header.frame_id = "{}_{:02x}".format(self._frame_prefix, addr)
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = _FIELD_OF_VIEW_RAD
            msg.min_range = _MIN_RANGE_M
            msg.max_range = _MAX_RANGE_M
            # math.inf = no target detected (sensor_msgs/Range convention)
            msg.range = math.inf if math.isinf(distance_mm) else distance_mm / 1000.0

            self._pubs[addr].publish(msg)

    # ------------------------------------------------------------------
    # Service: change I2C address
    # ------------------------------------------------------------------

    def _handle_change_address(
        self,
        request: ChangeAddress.Request,
        response: ChangeAddress.Response,
    ) -> ChangeAddress.Response:
        old_addr = request.current_address
        new_addr = request.new_address

        if old_addr not in self._sensors:
            response.success = False
            response.message = "No sensor at 0x{:02X}".format(old_addr)
            self.get_logger().warn(response.message)
            return response

        if new_addr in self._sensors:
            response.success = False
            response.message = "Address 0x{:02X} already in use".format(new_addr)
            self.get_logger().warn(response.message)
            return response

        sensor = self._sensors[old_addr]
        try:
            sensor.change_address(new_addr)
        except (Vl53l0xError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
            return response

        # Migrate bookkeeping to new address
        self.destroy_publisher(self._pubs.pop(old_addr))
        del self._sensors[old_addr]
        self._register_sensor(new_addr)

        response.success = True
        response.message = "0x{:02X} → 0x{:02X} OK".format(old_addr, new_addr)
        self.get_logger().info(response.message)
        return response

    def destroy_node(self) -> None:
        for sensor in self._sensors.values():
            sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Vl53l0xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
