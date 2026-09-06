#!/usr/bin/env python3
"""Read raw serial output from the ESP32-C3 without needing a real TTY
(idf.py monitor refuses to run when stdin isn't a terminal, e.g. under
nohup/background). Usage:

    read_serial.py [seconds] [--reset]

--reset toggles RTS/DTR first to force a fresh boot; without it, this just
observes whatever the device is currently doing (safe to run mid-commissioning).
"""
import serial, sys, time

PORT = "/dev/cu.usbmodem1101"
BAUD = 115200

seconds = 90
reset = False
for arg in sys.argv[1:]:
    if arg == "--reset":
        reset = True
    else:
        seconds = int(arg)

ser = serial.Serial(PORT, BAUD, timeout=1)
if reset:
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    time.sleep(0.1)

end = time.time() + seconds
while time.time() < end:
    data = ser.read(4096)
    if data:
        sys.stdout.buffer.write(data)
        sys.stdout.flush()
ser.close()
