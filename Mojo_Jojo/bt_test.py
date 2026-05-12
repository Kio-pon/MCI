#!/usr/bin/env python3
"""Simple Bluetooth serial test for HC-06."""

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:
    print("pyserial is not installed. Run: pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from exc


def read_available(ser, duration_s):
    """Read any bytes available for a short duration."""
    end_time = time.time() + duration_s
    chunks = []
    while time.time() < end_time:
        waiting = ser.in_waiting
        if waiting:
            chunks.append(ser.read(waiting))
        time.sleep(0.05)
    return b"".join(chunks)


def main():
    parser = argparse.ArgumentParser(description="HC-06 Bluetooth serial test")
    parser.add_argument("port", help="Serial port (e.g. /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument("--send", default="S", help="Message to send")
    parser.add_argument("--wait", type=float, default=1.0, help="Seconds to wait for response")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    with ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        payload = args.send.encode("ascii", errors="ignore")
        ser.write(payload)
        ser.flush()
        data = read_available(ser, args.wait)

    if data:
        print("Received:", data.decode("ascii", errors="replace"))
    else:
        print("No response received.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
