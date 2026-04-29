"""
Diagnostic script — run without ROS2 to verify I2C communication.

Usage:
  python3 debug_scan.py [bus] [addr_hex]

Examples:
  python3 debug_scan.py           # bus=3, addr=0x18
  python3 debug_scan.py 3 0x57   # custom address
"""

import sys
import time
import smbus2


def main():
    bus_id = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    addr = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x18

    print(f"Opening /dev/i2c-{bus_id}, probing 0x{addr:02X}")
    bus = smbus2.SMBus(bus_id)

    print("\n--- Raw read (10 attempts) ---")
    for i in range(10):
        try:
            write_msg = smbus2.i2c_msg.write(addr, [0x01])
            bus.i2c_rdwr(write_msg)
            time.sleep(0.120)
            read_msg = smbus2.i2c_msg.read(addr, 3)
            bus.i2c_rdwr(read_msg)
            data = list(read_msg)
            high, low, chk = data
            distance = (high << 8) | low
            expected_chk = (high + low) & 0xFF
            chk_ok = "OK" if chk == expected_chk else f"BAD (expected {expected_chk:#04x})"
            print(f"  [{i+1:2d}] bytes={[f'{b:#04x}' for b in data]}  "
                  f"distance={distance} mm  checksum={chk_ok}")
        except OSError as e:
            print(f"  [{i+1:2d}] OSError: {e}")
        time.sleep(0.1)

    bus.close()


if __name__ == "__main__":
    main()
