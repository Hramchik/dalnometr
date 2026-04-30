"""
Diagnostic script — run without ROS2 to verify I2C communication.

Frame format: [header=0x02, dist_high, dist_low, checksum]
No-echo sentinel: dist_high=0xFF, dist_low=0xFF (0xFFFF)

Usage:
  python3 debug_scan.py [bus] [addr_hex]

Examples:
  python3 debug_scan.py           # bus=1, addr=0x74
  python3 debug_scan.py 1 0x57   # custom address
"""

import math
import sys
import time
import smbus2


def main():
    bus_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    addr = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x74

    print(f"Opening /dev/i2c-{bus_id}, probing 0x{addr:02X}")
    bus = smbus2.SMBus(bus_id)

    print("\n--- Raw read (10 attempts) ---")
    for i in range(10):
        try:
            write_msg = smbus2.i2c_msg.write(addr, [0x01])
            bus.i2c_rdwr(write_msg)
            time.sleep(0.200)
            read_msg = smbus2.i2c_msg.read(addr, 4)
            bus.i2c_rdwr(read_msg)
            data = list(read_msg)
            header, high, low, chk = data
            raw = (high << 8) | low
            if raw == 0xFFFF:
                dist_str = "NO ECHO"
            else:
                dist_str = f"{raw} mm"
            header_ok = "OK" if header == 0x02 else f"UNEXPECTED (0x{header:02x})"
            print(f"  [{i+1:2d}] bytes={[f'{b:#04x}' for b in data]}  "
                  f"header={header_ok}  distance={dist_str}")
        except OSError as e:
            print(f"  [{i+1:2d}] OSError: {e}")
        time.sleep(0.1)

    bus.close()


if __name__ == "__main__":
    main()
