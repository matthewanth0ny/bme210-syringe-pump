#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  BME 210 Design Competition 2026 - Fluid Transport Sequence
#  Pulls water from Cup A, deposits into Cup B, repeats.
#
#  Flow per cycle:
#    Home → reach forward/down to Cup A → aspirate → back to Home
#    Home → rotate 90° and reach down to Cup B → dispense → back to Home
#
#  Both meArm servos and stepper share the same PCA9685 at address 0x60.
#  ONE PCA9685 object is used for everything — no MotorKit, no object
#  recreation. Frequency is switched directly on the same object and
#  servo duty cycles are saved/restored around the switch.

import board
import busio
import json
import logging
import os
import time
import meArm
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# --- Load meArm calibration from config ---
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "mearm_config.json")
with open(CONFIG_PATH, "r") as f:
    config = json.load(f)

BASE_MID = config["base"]["zero"]
SHOULDER_MID = config["shoulder"]["zero"]
ELBOW_MID = config["elbow"]["zero"]
GRIPPER_MID = config["gripper"]["zero"]

# --- Constants ---
I2C_ADDRESS = 0x60

# Stepper settings
STEPS_PER_SECOND = 3500  # push the I2C bus harder for faster aspirate/dispense
STEP_DELAY = 1.0 / STEPS_PER_SECOND
HOMING_BUFFER = 40  # small safety overshoot toward min, enough to home but not grind

# Stepper directions
DIR_TOWARD_MIN = -1  # homing / dispensing (push plunger out)
DIR_TOWARD_MAX = 1   # aspirating (pull plunger in)

# Load stepper calibration (min = empty, max = full)
STEPPER_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "stepper_config.json")
with open(STEPPER_CONFIG_PATH, "r") as f:
    stepper_config = json.load(f)
STEPPER_MIN = stepper_config["min"]
STEPPER_MAX = stepper_config["max"]
SYRINGE_STEPS = abs(STEPPER_MAX - STEPPER_MIN)  # total travel between min and max

# Load end-of-sequence push poses (tuned via arm_extend.py)
PUSH_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "arm_extend_config.json")
# Live-run nudges (degrees) added on top of the saved arm_extend_config.json
# values so you can dial in physical motion without re-tuning.
#
# BEFORE_PUSH = lowering down to the cup → bias the ELBOW
# PUSH        = extending forward to push   → bias the SHOULDER
#
# Sign convention (+ = larger angle written to that channel):
#   BEFORE_PUSH_ELBOW_NUDGE  positive → elbow rotates physically lower
#   PUSH_SHOULDER_NUDGE      positive → shoulder rotates physically lower
BEFORE_PUSH_ELBOW_NUDGE = 25
PUSH_SHOULDER_NUDGE     = 25

push_poses = None
if os.path.exists(PUSH_CONFIG_PATH):
    with open(PUSH_CONFIG_PATH, "r") as f:
        push_poses = json.load(f)

# --- Positions ---
# Base sweeps from its calibrated min to max (full 180°).
# Shoulder and elbow still lower/raise the arm into each cup.
# Cup A = base at min (-90° from home), Cup B = base at max (+90° from home)
# Cup z is the "lowered into cup" height. Lower this to dip deeper into the cup.
# (kinematics may refuse very low values — start with small drops.)
CUP_Z = 100
CUP_A = (-150, 0, CUP_Z)   # base at min
CUP_B = (150, 0, CUP_Z)    # base at max

# Home height (raised for safe travel — must clear cup rim)
HOME_Z = 200

NUM_CYCLES = 4

# --- PCA9685 channel mapping ---
# meArm servo channels
SERVO_CHANNELS = [0, 1, 14, 15]  # base, shoulder, elbow, gripper

# Motor HAT M1+M2 stepper channels
M1_PWM = 8
M1_IN2 = 9
M1_IN1 = 10
M2_PWM = 13
M2_IN2 = 12
M2_IN1 = 11
STEPPER_CHANNELS = [M1_PWM, M1_IN1, M1_IN2, M2_PWM, M2_IN1, M2_IN2]

FULL = 0xFFFF
OFF = 0

# DOUBLE step sequence (both coils energized for more torque)
# Each step: (IN1_A, IN2_A, IN1_B, IN2_B)
STEP_SEQUENCE = [
    (FULL, OFF,  FULL, OFF),   # A+, B+
    (OFF,  FULL, FULL, OFF),   # A-, B+
    (OFF,  FULL, OFF,  FULL),  # A-, B-
    (FULL, OFF,  OFF,  FULL),  # A+, B-
]

# --- Single shared PCA9685 object (never recreated) ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=I2C_ADDRESS, reference_clock_speed=25000000)
pca.frequency = 50  # start in servo mode

# Track stepper position in sequence
step_index = 0

# Servo objects on the shared pca (used only for the final extend/push)
ARM_SERVOS = {
    "base":     servo.Servo(pca.channels[0],  min_pulse=500, max_pulse=2500),
    "shoulder": servo.Servo(pca.channels[1],  min_pulse=500, max_pulse=2500),
    "elbow":    servo.Servo(pca.channels[14], min_pulse=500, max_pulse=2500),
    "gripper": servo.Servo(pca.channels[15], min_pulse=500, max_pulse=2500),
}


def apply_named_pose(pose_dict, settle=1.0):
    for name, deg in pose_dict.items():
        clamped = max(0.0, min(180.0, float(deg)))
        print(f"      writing {name:>8} -> {clamped:.1f}° (raw={deg:.1f})")
        ARM_SERVOS[name].angle = clamped
    time.sleep(settle)


def do_final_push():
    """Run the tuned HOME → BEFORE_PUSH → PUSH sequence at Cup B.

    HOME is applied first via the SAME servo library used by arm_extend.py
    so the arm is in the same reference frame the poses were tuned in
    (the meArm library may map angles to different pulse widths)."""
    if not push_poses:
        print("  (no arm_extend_config.json found — skipping final push)")
        return
    # Stay at Cup B's base angle for the push — drop "base" from the poses
    # so we don't rotate the arm away from cup B.
    print("  BEFORE_PUSH (lowering — elbow nudge)...")
    before = {k: v for k, v in push_poses["before_push"].items() if k != "base"}
    saved_elbow = before.get("elbow", 90)
    before["elbow"] = saved_elbow + BEFORE_PUSH_ELBOW_NUDGE
    print(f"    elbow:    saved={saved_elbow:.1f}  +nudge={BEFORE_PUSH_ELBOW_NUDGE} -> {before['elbow']:.1f}")
    apply_named_pose(before)
    print("  PUSH (extending forward — shoulder nudge)...")
    push = {k: v for k, v in push_poses["push"].items() if k != "base"}
    saved_shoulder = push.get("shoulder", 90)
    push["shoulder"] = saved_shoulder + PUSH_SHOULDER_NUDGE
    print(f"    shoulder: saved={saved_shoulder:.1f}  +nudge={PUSH_SHOULDER_NUDGE} -> {push['shoulder']:.1f}")
    apply_named_pose(push)


# --- Servo save/restore ---

def save_servo_state():
    """Save current duty_cycle of all servo channels."""
    saved = {}
    for ch in SERVO_CHANNELS:
        saved[ch] = pca.channels[ch].duty_cycle
    return saved


def zero_servo_channels():
    """De-energize all servo channels (servos go limp, no garbage signal)."""
    for ch in SERVO_CHANNELS:
        pca.channels[ch].duty_cycle = 0
    time.sleep(0.05)


def restore_servo_state(saved_state):
    """Write back saved duty_cycle values to servo channels."""
    for ch, val in saved_state.items():
        pca.channels[ch].duty_cycle = val
    time.sleep(0.3)


# --- Frequency switching ---

def switch_to_1600():
    """Switch PCA9685 to 1600 Hz (stepper mode).
    Servo channels must already be zeroed before calling this."""
    pca.frequency = 1600
    time.sleep(0.05)


def switch_to_50():
    """Switch PCA9685 back to 50 Hz (servo mode).
    Stepper channels must already be zeroed before calling this."""
    pca.frequency = 50
    time.sleep(0.05)


# --- Stepper control (direct PCA9685, no MotorKit) ---

def stepper_step(direction):
    """Advance the stepper one step via H-bridge channels."""
    global step_index
    if direction == 1:   # forward
        step_index = (step_index + 1) % 4
    else:                # backward
        step_index = (step_index - 1) % 4

    in1_a, in2_a, in1_b, in2_b = STEP_SEQUENCE[step_index]

    pca.channels[M1_PWM].duty_cycle = FULL
    pca.channels[M1_IN1].duty_cycle = in1_a
    pca.channels[M1_IN2].duty_cycle = in2_a
    pca.channels[M2_PWM].duty_cycle = FULL
    pca.channels[M2_IN1].duty_cycle = in1_b
    pca.channels[M2_IN2].duty_cycle = in2_b


def run_stepper_steps(direction, num_steps):
    """Run stepper for a fixed number of steps. direction: 1=fwd, -1=back."""
    for _ in range(num_steps):
        stepper_step(direction)
        time.sleep(STEP_DELAY)


def release_stepper():
    """De-energize all stepper coils."""
    for ch in STEPPER_CHANNELS:
        pca.channels[ch].duty_cycle = OFF


def do_syringe(direction, num_steps):
    """Full syringe operation: save servos → switch to 1600 Hz → step →
    release → switch back to 50 Hz → restore servos."""
    saved = save_servo_state()
    zero_servo_channels()
    switch_to_1600()

    run_stepper_steps(direction, num_steps)
    release_stepper()

    switch_to_50()
    restore_servo_state(saved)


def home_stepper():
    """Drive stepper backward to the min position (hard stop).
    Extra steps past max ensure it reaches the min end regardless
    of where it started. Steps are 'lost' harmlessly at the hard stop."""
    print("Homing stepper to min position...")
    saved = save_servo_state()
    zero_servo_channels()
    switch_to_1600()

    run_stepper_steps(DIR_TOWARD_MIN, SYRINGE_STEPS + HOMING_BUFFER)
    release_stepper()

    switch_to_50()
    restore_servo_state(saved)
    print("Stepper homed.")


# --- Main sequence ---

# Initialize meArm ONCE using config — this is the calibrated home
arm = meArm.meArm(address=I2C_ADDRESS, logger=logger)

# Home the stepper to its min (fully retracted) position
home_stepper()

input("\nPress Enter to begin competition sequence...")

try:
    for cycle in range(NUM_CYCLES):
        print(f"\n--- Cycle {cycle + 1} of {NUM_CYCLES} ---")

        # ==============================
        # Cup A: rotate over and drop in one motion
        # ==============================
        print("  Moving to Cup A (base min)...")
        arm.move_linear(CUP_A[0], CUP_A[1], HOME_Z)    # travel high to cup A
        arm.move_linear(CUP_A[0], CUP_A[1], CUP_A[2])  # lower into cup

        # Aspirate (pull water) — drive from min → max
        print(f"  Aspirating ({SYRINGE_STEPS} steps, min → max)...")
        do_syringe(DIR_TOWARD_MAX, SYRINGE_STEPS)

        # ==============================
        # Cup B: combined raise + rotate + lower (no intermediate stop)
        # ==============================
        print("  Moving to Cup B (base max)...")
        arm.move_linear(CUP_B[0], CUP_B[1], HOME_Z)    # diagonal lift+rotate
        arm.move_linear(CUP_B[0], CUP_B[1], CUP_B[2])  # lower into cup

        # Dispense (push water) — drive from max → min
        print(f"  Dispensing ({SYRINGE_STEPS} steps, max → min)...")
        do_syringe(DIR_TOWARD_MIN, SYRINGE_STEPS)

        is_last_cycle = (cycle == NUM_CYCLES - 1)

        if is_last_cycle:
            print("  Final cycle — executing end push at Cup B...")
            do_final_push()
        else:
            # Travel directly back over Cup A in one diagonal motion
            arm.move_linear(CUP_A[0], CUP_A[1], HOME_Z)

    print("\nSequence complete!")

finally:
    release_stepper()
