"""
ROS2 Humble node for DYP-A22 (AA2211AC) ultrasonic sensors on I2C.

Published topics (one per sensor):
  ~/sensor_<hex_addr>   sensor_msgs/Range

Services:
  ~/change_address      dalnometr_msgs/srv/ChangeAddress

Parameters:
  i2c_bus          (int,   default 1)       — Linux I2C bus index (/dev/i2c-N)
  addresses        (int[], default [87])    — addresses to probe on startup (decimal)
  scan_all_7bit    (bool,  default false)   — probe full 7-bit space (0x08–0x77)
  publish_rate_hz  (float, default 10.0)   — measurement frequency per sensor
  frame_prefix     (str,   default "ultrasonic") — tf frame_id prefix
"""

import rclpy
from rclpy.node import Node
import smbus2
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from dalnometr_msgs.srv import ChangeAddress
from dalnometr_ros2.dyp_a22_driver import DypA22, DypA22Error, scan_bus

# DYP-A22 physical beam characteristics
_FIELD_OF_VIEW_RAD = 0.2618  # ~15 degrees
_MIN_RANGE_M = 0.020
_MAX_RANGE_M = 4.500


class DypA22Node(Node):
    def __init__(self):
        super().__init__("dyp_a22_node")

        self._declare_parameters()

        bus_id: int = self.get_parameter("i2c_bus").value
        addresses: list = list(self.get_parameter("addresses").value)
        scan_all: bool = self.get_parameter("scan_all_7bit").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._frame_prefix: str = self.get_parameter("frame_prefix").value

        if scan_all:
            addresses = list(range(0x08, 0x78))

        try:
            self._bus = smbus2.SMBus(bus_id)
        except Exception as exc:
            self.get_logger().fatal(f"Cannot open /dev/i2c-{bus_id}: {exc}")
            raise SystemExit(1)

        self.get_logger().info(
            "Scanning /dev/i2c-{} at: {}".format(
                bus_id, ", ".join(f"0x{a:02X}" for a in addresses)
            )
        )

        found = scan_bus(self._bus, addresses)
        if not found:
            self.get_logger().warn("No DYP-A22 sensors detected on I2C bus.")

        self._sensors: dict = {}
        self._publishers: dict = {}

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
        self.declare_parameter("addresses", [0x57])   # 87 decimal
        self.declare_parameter("scan_all_7bit", False)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_prefix", "ultrasonic")

    # ------------------------------------------------------------------
    # Sensor registration
    # ------------------------------------------------------------------

    def _register_sensor(self, address: int) -> None:
        sensor = DypA22(self._bus, address)
        topic = "sensor_{:02x}".format(address)
        pub = self.create_publisher(Range, "~/{}".format(topic), 10)
        self._sensors[address] = sensor
        self._publishers[address] = pub
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
            except DypA22Error as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
                continue

            msg = Range()
            msg.header = Header()
            msg.header.stamp = now
            msg.header.frame_id = "{}_{:02x}".format(self._frame_prefix, addr)
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = _FIELD_OF_VIEW_RAD
            msg.min_range = _MIN_RANGE_M
            msg.max_range = _MAX_RANGE_M
            msg.range = distance_mm / 1000.0  # mm → m

            self._publishers[addr].publish(msg)

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
        except (DypA22Error, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
            return response

        # Migrate bookkeeping to new address
        self.destroy_publisher(self._publishers.pop(old_addr))
        del self._sensors[old_addr]
        self._register_sensor(new_addr)

        response.success = True
        response.message = "0x{:02X} → 0x{:02X} OK".format(old_addr, new_addr)
        self.get_logger().info(response.message)
        return response

    def destroy_node(self) -> None:
        if hasattr(self, "_bus"):
            self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DypA22Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
