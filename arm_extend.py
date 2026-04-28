#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Interactive tuner + runner for the end-of-sequence cup push.
#  Lets you dial in BEFORE_PUSH and PUSH angles on YOUR physical arm,
#  save them to arm_extend_config.json, and play the sequence.
#
#  Uses the same servo library as Zero.py so calibration matches exactly.
#  HOME values come from mearm_config.json.

import board
import json
import os
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Servo channels ---
BASE = 0
SHOULDER = 1
ELBOW = 14
GRIPPER = 15

CHANNEL_NAMES = {BASE: "base", SHOULDER: "shoulder", ELBOW: "elbow", GRIPPER: "gripper"}
NAME_TO_CH = {v: k for k, v in CHANNEL_NAMES.items()}

# --- Load HOME calibration ---
HERE = os.path.dirname(__file__)
MEARM_CONFIG = os.path.join(HERE, "mearm_config.json")
PUSH_CONFIG = os.path.join(HERE, "arm_extend_config.json")

with open(MEARM_CONFIG, "r") as f:
    cfg = json.load(f)

HOME = {
    BASE:     cfg["base"]["zero"],
    SHOULDER: cfg["shoulder"]["zero"],
    ELBOW:    cfg["elbow"]["zero"],
    GRIPPER:  cfg["gripper"]["zero"],
}

# Defaults if no saved config yet — overwritten by file if present
BEFORE_PUSH = dict(HOME)
PUSH = dict(HOME)

if os.path.exists(PUSH_CONFIG):
    with open(PUSH_CONFIG, "r") as f:
        saved = json.load(f)
    for name, ch in NAME_TO_CH.items():
        if name in saved.get("before_push", {}):
            BEFORE_PUSH[ch] = saved["before_push"][name]
        if name in saved.get("push", {}):
            PUSH[ch] = saved["push"][name]

SETTLE = 0.8

# --- PCA9685 setup (same as Zero.py) ---
i2c = board.I2C()
pca = PCA9685(i2c, address=0x60, reference_clock_speed=25000000)
pca.frequency = 50

SERVOS = {
    BASE:     servo.Servo(pca.channels[BASE],     min_pulse=500, max_pulse=2500),
    SHOULDER: servo.Servo(pca.channels[SHOULDER], min_pulse=500, max_pulse=2500),
    ELBOW:    servo.Servo(pca.channels[ELBOW],    min_pulse=500, max_pulse=2500),
    GRIPPER:  servo.Servo(pca.channels[GRIPPER],  min_pulse=500, max_pulse=2500),
}


def clamp(v, lo=0.0, hi=180.0):
    return max(lo, min(hi, v))


def apply_pose(pose):
    for ch, deg in pose.items():
        SERVOS[ch].angle = clamp(deg)


def show(pose, label):
    print(f"  {label}:")
    for ch in (BASE, SHOULDER, ELBOW, GRIPPER):
        print(f"    {CHANNEL_NAMES[ch]:>8}: {pose[ch]:.1f}°")


def save():
    out = {
        "before_push": {CHANNEL_NAMES[ch]: BEFORE_PUSH[ch] for ch in BEFORE_PUSH},
        "push":        {CHANNEL_NAMES[ch]: PUSH[ch]        for ch in PUSH},
    }
    with open(PUSH_CONFIG, "w") as f:
        json.dump(out, f, indent=4)
    print(f"  Saved → {PUSH_CONFIG}")


def tune(pose, label):
    """Interactively adjust each joint of `pose`. Live-applies on every change."""
    print(f"\n--- Tuning {label} ---")
    print("Commands:")
    print("  <joint> <deg>      set absolute angle (e.g. 'shoulder 90')")
    print("  <joint> +<n>       nudge by +n  (e.g. 'elbow +5')")
    print("  <joint> -<n>       nudge by -n")
    print("  show               print current pose")
    print("  home               apply HOME (does not change tuned pose)")
    print("  apply              re-apply this pose")
    print("  done               finish tuning this pose")
    apply_pose(pose)
    show(pose, label)
    while True:
        cmd = input(f"[{label}] > ").strip().lower()
        if not cmd:
            continue
        if cmd == "done":
            return
        if cmd == "show":
            show(pose, label); continue
        if cmd == "home":
            apply_pose(HOME); continue
        if cmd == "apply":
            apply_pose(pose); continue
        parts = cmd.split()
        if len(parts) != 2 or parts[0] not in NAME_TO_CH:
            print("  ?  try: shoulder 90  |  elbow +5  |  done")
            continue
        ch = NAME_TO_CH[parts[0]]
        arg = parts[1]
        try:
            if arg.startswith(("+", "-")):
                pose[ch] = clamp(pose[ch] + float(arg))
            else:
                pose[ch] = clamp(float(arg))
        except ValueError:
            print("  bad number"); continue
        SERVOS[ch].angle = pose[ch]
        print(f"    {parts[0]} = {pose[ch]:.1f}°")


# --- Main ---
print("Going HOME...")
apply_pose(HOME)
time.sleep(SETTLE)

while True:
    print("\n=== arm_extend ===")
    print("  1) tune BEFORE_PUSH (lowered)")
    print("  2) tune PUSH (extended forward)")
    print("  3) play sequence: HOME → BEFORE_PUSH → PUSH → HOME")
    print("  4) save to arm_extend_config.json")
    print("  5) show both poses")
    print("  q) quit")
    choice = input("> ").strip().lower()
    if choice == "1":
        tune(BEFORE_PUSH, "BEFORE_PUSH")
    elif choice == "2":
        tune(PUSH, "PUSH")
    elif choice == "3":
        print("HOME...");        apply_pose(HOME);        time.sleep(SETTLE)
        print("BEFORE_PUSH..."); apply_pose(BEFORE_PUSH); time.sleep(SETTLE)
        print("PUSH...");        apply_pose(PUSH);        time.sleep(SETTLE)
        print("HOME...");        apply_pose(HOME);        time.sleep(SETTLE)
    elif choice == "4":
        save()
    elif choice == "5":
        show(HOME, "HOME")
        show(BEFORE_PUSH, "BEFORE_PUSH")
        show(PUSH, "PUSH")
    elif choice == "q":
        break
    else:
        print("  ?")

print("Done.")
