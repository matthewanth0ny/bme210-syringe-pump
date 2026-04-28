#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Stepper Motor Calibration
#  Jog the stepper to find the min (empty) and max (full) positions.
#  Saves step counts to stepper_config.json.
#
#  Controls:
#    f / <Enter>   = step forward (push)
#    b             = step backward (pull)
#    F             = step forward fast (100 steps)
#    B             = step backward fast (100 steps)
#    min           = save current position as MIN (empty syringe)
#    max           = save current position as MAX (full syringe)
#    zero          = reset counter to 0
#    q             = quit and save config

import board
import busio
import json
import os
import time
from adafruit_pca9685 import PCA9685

I2C_ADDRESS = 0x60
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "stepper_config.json")

# Motor HAT M1+M2 stepper channels
M1_PWM, M1_IN2, M1_IN1 = 8, 9, 10
M2_PWM, M2_IN2, M2_IN1 = 13, 12, 11

FULL = 0xFFFF
OFF = 0

STEP_SEQUENCE = [
    (FULL, OFF,  FULL, OFF),
    (OFF,  FULL, FULL, OFF),
    (OFF,  FULL, OFF,  FULL),
    (FULL, OFF,  OFF,  FULL),
]

STEP_DELAY = 0.002  # 500 steps/sec

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=I2C_ADDRESS)
pca.frequency = 1600

step_index = 0
position = 0  # total steps from starting point


def do_step(direction):
    global step_index, position
    step_index = (step_index + direction) % 4
    in1_a, in2_a, in1_b, in2_b = STEP_SEQUENCE[step_index]
    pca.channels[M1_PWM].duty_cycle = FULL
    pca.channels[M1_IN1].duty_cycle = in1_a
    pca.channels[M1_IN2].duty_cycle = in2_a
    pca.channels[M2_PWM].duty_cycle = FULL
    pca.channels[M2_IN1].duty_cycle = in1_b
    pca.channels[M2_IN2].duty_cycle = in2_b
    position += direction
    time.sleep(STEP_DELAY)


def step_many(direction, count):
    for _ in range(count):
        do_step(direction)


def release():
    for ch in [M1_PWM, M1_IN1, M1_IN2, M2_PWM, M2_IN1, M2_IN2]:
        pca.channels[ch].duty_cycle = OFF


# Load existing config if present
config = {"min": None, "max": None}
if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)

print("=" * 50)
print("Stepper Motor Calibration")
print("=" * 50)
print("Commands:")
print("  f or <Enter>  = step forward 1")
print("  b             = step backward 1")
print("  F             = forward 100 steps")
print("  B             = backward 100 steps")
print("  ff <n>        = forward n steps")
print("  bb <n>        = backward n steps")
print("  min           = save current position as MIN")
print("  max           = save current position as MAX")
print("  zero          = reset counter to 0")
print("  show          = show current config")
print("  q             = quit and save")
print("=" * 50)

try:
    while True:
        cmd = input(f"[pos={position}, min={config['min']}, max={config['max']}] > ").strip()

        if cmd == "" or cmd == "f":
            do_step(1)
        elif cmd == "b":
            do_step(-1)
        elif cmd == "F":
            step_many(1, 100)
        elif cmd == "B":
            step_many(-1, 100)
        elif cmd.startswith("ff "):
            step_many(1, int(cmd.split()[1]))
        elif cmd.startswith("bb "):
            step_many(-1, int(cmd.split()[1]))
        elif cmd == "min":
            config["min"] = position
            print(f"  Saved MIN = {position}")
        elif cmd == "max":
            config["max"] = position
            print(f"  Saved MAX = {position}")
        elif cmd == "zero":
            position = 0
            print("  Counter reset to 0")
        elif cmd == "show":
            print(f"  Current config: {config}")
        elif cmd == "q":
            break
        else:
            print("  Unknown command")

finally:
    release()
    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=4)
    print(f"\nConfig saved to {CONFIG_PATH}")
    print(f"Final: {config}")
