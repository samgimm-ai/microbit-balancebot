"""
TB6612FNG Motor Driver for BBC micro:bit
Controls two DC gear motors (1:220 gear ratio) via Keyes KE0136 expansion board
"""

from microbit import pin1, pin2, pin12, pin13, pin15, pin16


class Motor:
    """Single DC motor controller."""

    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin

        # Initialize pins
        self.pwm_pin.write_analog(0)
        self.in1_pin.write_digital(0)
        self.in2_pin.write_digital(0)

    def forward(self, speed):
        """Run motor forward at given speed (0-1023)."""
        speed = min(1023, max(0, int(speed)))
        self.in1_pin.write_digital(1)
        self.in2_pin.write_digital(0)
        self.pwm_pin.write_analog(speed)

    def backward(self, speed):
        """Run motor backward at given speed (0-1023)."""
        speed = min(1023, max(0, int(speed)))
        self.in1_pin.write_digital(0)
        self.in2_pin.write_digital(1)
        self.pwm_pin.write_analog(speed)

    def stop(self):
        """Coast stop (free spin)."""
        self.in1_pin.write_digital(0)
        self.in2_pin.write_digital(0)
        self.pwm_pin.write_analog(0)

    def brake(self):
        """Active brake (short brake)."""
        self.in1_pin.write_digital(1)
        self.in2_pin.write_digital(1)
        self.pwm_pin.write_analog(0)

    def drive(self, speed):
        """
        Drive motor at signed speed.
        Positive = forward, negative = backward, 0 = stop.

        Args:
            speed: -1023 to +1023
        """
        if speed > 0:
            self.forward(speed)
        elif speed < 0:
            self.backward(-speed)
        else:
            self.stop()


class DualMotor:
    """Controls both motors for the balancing robot."""

    def __init__(self):
        # Motor 1 (left): PWM=P1, IN1=P13, IN2=P12
        self.left = Motor(pin1, pin13, pin12)
        # Motor 2 (right): PWM=P2, IN1=P15, IN2=P16
        self.right = Motor(pin2, pin15, pin16)

    def drive(self, speed):
        """Drive both motors at same signed speed."""
        self.left.drive(speed)
        self.right.drive(speed)

    def drive_lr(self, left_speed, right_speed):
        """Drive left and right motors independently."""
        self.left.drive(left_speed)
        self.right.drive(right_speed)

    def stop(self):
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def brake(self):
        """Brake both motors."""
        self.left.brake()
        self.right.brake()
