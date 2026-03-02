"""
Micro:bit Self-Balancing Robot - Main Control Loop

Hardware:
  - BBC micro:bit V2
  - Keyes KE0136 motor driver board (TB6612FNG)
  - MPU6050 (GY-521) 6-axis IMU via I2C
  - 2x DC gear motors (1:220 gear ratio)
  - micro:bit battery pack (AAA x2, 3V) — micro:bit dedicated power
  - 18650 Battery Shield (5V output) — motor power only
"""

from microbit import display, Image, button_a, button_b, running_time, sleep
from mpu6050 import MPU6050
from motor import DualMotor
from pid import PID

# Constants
TARGET_ANGLE = 0.0      # Target balance angle (degrees)
FALL_THRESHOLD = 45.0   # Stop motors if tilted beyond this (degrees)
LOOP_MS = 5             # Control loop period (5ms = 200Hz)

# LED images for tilt direction
IMG_LEVEL = Image("00000:00000:99999:00000:00000")
IMG_FWD = Image("00900:09990:99999:00000:00000")
IMG_BWD = Image("00000:00000:99999:09990:00900")
IMG_FALL = Image("90009:09090:00900:09090:90009")  # X mark for fallen


def show_tilt(angle):
    """Show tilt direction on LED matrix."""
    if abs(angle) < 3:
        display.show(IMG_LEVEL)
    elif angle > 0:
        display.show(IMG_FWD)
    else:
        display.show(IMG_BWD)


def main():
    display.show(Image.HAPPY)
    sleep(500)

    # Initialize MPU6050
    display.scroll("IMU", delay=60)
    imu = MPU6050()

    # Calibrate (robot must be upright and still)
    display.scroll("CAL", delay=60)
    imu.calibrate(samples=200)

    # Initialize motors
    motors = DualMotor()
    motors.stop()

    # Initialize PID controller
    pid = PID(kp=100.0, ki=76.0, kd=52.0)

    display.show(Image.YES)
    sleep(500)

    # PID tuning mode
    # Button A: cycle through Kp/Ki/Kd selection
    # Button B: increase selected parameter by 5
    tune_param = 0  # 0=Kp, 1=Ki, 2=Kd
    param_names = ["P", "I", "D"]

    prev_time = running_time()
    fallen = False

    while True:
        # Timing
        now = running_time()
        dt_ms = now - prev_time
        if dt_ms < LOOP_MS:
            sleep(LOOP_MS - dt_ms)
            now = running_time()
            dt_ms = now - prev_time
        prev_time = now
        dt = dt_ms / 1000.0  # Convert to seconds

        # Read filtered angle
        angle = imu.get_angle(dt)

        # Fall detection
        if abs(angle) > FALL_THRESHOLD:
            if not fallen:
                motors.brake()
                pid.reset()
                display.show(IMG_FALL)
                fallen = True
            # Check for recovery (picked back up)
            if abs(angle) < FALL_THRESHOLD - 10:
                fallen = False
            continue

        if fallen:
            fallen = False
            pid.reset()

        # PID computation
        error = TARGET_ANGLE - angle
        output = pid.compute(error, dt)

        # Drive motors (both same speed for balance)
        motors.drive(int(output))

        # Update LED display (throttled to avoid I2C contention)
        if now % 200 < LOOP_MS:
            show_tilt(angle)

        # Button A: select PID parameter to tune
        if button_a.was_pressed():
            tune_param = (tune_param + 1) % 3
            display.scroll(param_names[tune_param], delay=50, wait=False)

        # Button B: increase selected parameter
        if button_b.was_pressed():
            if tune_param == 0:
                pid.kp += 5
            elif tune_param == 1:
                pid.ki += 2
            elif tune_param == 2:
                pid.kd += 2
            display.scroll(
                "{}{}".format(param_names[tune_param],
                              int([pid.kp, pid.ki, pid.kd][tune_param])),
                delay=50, wait=False
            )


main()
