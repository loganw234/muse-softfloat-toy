#!/usr/bin/env python3
"""Print Pico CDC lines until one contains MARKER.

Port is selected by USB serial (TOY_BOARD_SERIAL) so a multi-board
bus can never hand us the wrong device; the capture window is an
argument because suites outgrow fixed windows (learned the hard
way in the library project's bench round).

usage: read_toy.py MARKER WINDOW_SECONDS
"""
import os
import sys
import time

import serial
import serial.tools.list_ports


def find_port():
    want = os.environ.get("TOY_BOARD_PORT")
    if want:
        return want
    ser = os.environ.get("TOY_BOARD_SERIAL", "").lower()
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A and (not ser or (p.serial_number or "").lower() == ser):
            return p.device
    return None


def main():
    marker = sys.argv[1]
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
    port = None
    deadline = time.time() + 60
    while not port and time.time() < deadline:
        port = find_port()
        if not port:
            time.sleep(0.5)
    if not port:
        print("board: no matching Pico CDC port enumerated within 60s")
        return 2
    print("board port: %s" % port)
    ser = serial.Serial(port, 115200, timeout=10)
    t_end = time.time() + window
    while time.time() < t_end:
        line = ser.readline()
        if not line:
            continue
        s = line.decode("utf-8", "replace").strip()
        print(s, flush=True)
        if marker in s:
            return 0
    print("board: marker %r not seen within %ss" % (marker, window))
    return 1


if __name__ == "__main__":
    sys.exit(main())
