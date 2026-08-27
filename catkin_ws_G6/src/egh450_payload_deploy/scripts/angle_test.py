#!/usr/bin/env python3
from gpiozero import AngularServo
from time import sleep

M1 = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025)  # Tracker
M2 = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025)  # EpiPen

print("Angle test - no ROS needed")
print("Type a number to move M1 (Tracker), e.g: 70")
print("Type 'e' then a number to move M2 (EpiPen), e.g: e -40")
print("Type 'q' to quit")

while True:
    raw = input("> ").strip().lower()

    if raw == 'q':
        break

    try:
        if raw.startswith('e'):
            angle = float(raw[1:].strip())
            print(f"Moving M2 (EpiPen) to {angle}")
            M2.angle = angle
        else:
            angle = float(raw)
            print(f"Moving M1 (Tracker) to {angle}")
            M1.angle = angle
    except ValueError:
        print("Didn't understand that - type a number, 'e <number>', or 'q'")

M1.detach()
M2.detach()
print("Done, servos detached.")