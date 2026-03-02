"""
PID Controller for self-balancing robot
"""


class PID:
    def __init__(self, kp=100.0, ki=76.0, kd=52.0,
                 output_min=-1023, output_max=1023,
                 integral_limit=500):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_limit: Anti-windup integral term limit
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        """
        Compute PID output.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds

        Returns:
            Control output clamped to [output_min, output_max]
        """
        # Proportional term
        p = self.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        if self._integral > self.integral_limit:
            self._integral = self.integral_limit
        elif self._integral < -self.integral_limit:
            self._integral = -self.integral_limit
        i = self.ki * self._integral

        # Derivative term
        if dt > 0:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        d = self.kd * derivative

        self._prev_error = error

        # Sum and clamp output
        output = p + i + d
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        return output

    def reset(self):
        """Reset integral and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0
