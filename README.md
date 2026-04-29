# dalnometr — ROS2 Humble driver for DYP-A22 ultrasonic sensors

Supports the **DYP-A22 (AA2211AC)** ultrasonic distance sensor connected via I2C.

## Workspace structure

```
dalnometr/
├── dalnometr_msgs/        # Custom service definitions (CMake package)
│   └── srv/
│       └── ChangeAddress.srv
└── dalnometr_ros2/        # Sensor driver node (Python package)
    ├── dalnometr_ros2/
    │   ├── dyp_a22_driver.py   # Low-level I2C driver
    │   └── dyp_a22_node.py     # ROS2 node
    ├── launch/
    │   └── dyp_a22.launch.py
    └── test/
        └── test_dyp_a22_driver.py
```

## DYP-A22 I2C protocol

| Operation      | Details |
|----------------|---------|
| Default address | `0x57` |
| Start measurement | Write single byte `0x01` |
| Read result | 3 bytes: `[high, low, checksum]` |
| Distance | `(high << 8) \| low` in millimetres |
| Checksum | `(high + low) & 0xFF` |
| Change address | Write `[0x55, 0xAA, 0xA2, new_addr]` |

## Build

```bash
cd <ros2_ws>
ln -s /path/to/dalnometr/dalnometr_msgs   src/dalnometr_msgs
ln -s /path/to/dalnometr/dalnometr_ros2   src/dalnometr_ros2
colcon build --packages-select dalnometr_msgs dalnometr_ros2
source install/setup.bash
```

## Run

```bash
# Single sensor at default address 0x57 on /dev/i2c-1
ros2 launch dalnometr_ros2 dyp_a22.launch.py

# Two sensors at custom addresses
ros2 launch dalnometr_ros2 dyp_a22.launch.py addresses:=[87,88]

# Scan entire 7-bit range
ros2 launch dalnometr_ros2 dyp_a22.launch.py scan_all_7bit:=true

# Direct node launch with parameters
ros2 run dalnometr_ros2 dyp_a22_node \
  --ros-args -p i2c_bus:=1 -p addresses:=[87] -p publish_rate_hz:=20.0
```

## Published topics

Each detected sensor publishes `sensor_msgs/Range` on:

```
/sensors/dyp_a22/sensor_<hex_addr>
```

Example — sensor at `0x57`:
```
/sensors/dyp_a22/sensor_57
```

Fields:
- `radiation_type` = `ULTRASOUND`
- `field_of_view` = `0.2618 rad` (~15°)
- `min_range` = `0.020 m`
- `max_range` = `4.500 m`
- `range` — measured distance in metres

## Change I2C address at runtime

```bash
ros2 service call /sensors/dyp_a22/change_address \
  dalnometr_msgs/srv/ChangeAddress \
  "{current_address: 87, new_address: 88}"
```

After a successful call the node destroys the old publisher and creates a new
one on the new address immediately.

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `i2c_bus` | int | `1` | Linux I2C bus index (`/dev/i2c-N`) |
| `addresses` | int[] | `[87]` | Addresses to probe on startup (decimal) |
| `scan_all_7bit` | bool | `false` | Probe all valid 7-bit addresses |
| `publish_rate_hz` | float | `10.0` | Measurement frequency (Hz) |
| `frame_prefix` | string | `"ultrasonic"` | Prefix for `frame_id` in Range messages |

## Run unit tests (no hardware)

```bash
cd dalnometr_ros2
PYTHONPATH=. python3 -m pytest test/ -v
```
