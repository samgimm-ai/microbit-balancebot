"""
MPU6050 I2C Driver for BBC micro:bit
6-axis gyroscope/accelerometer with complementary filter
"""

from microbit import i2c, sleep
import math
import struct

MPU6050_ADDR = 0x68

# Register addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75


class MPU6050:
    def __init__(self, addr=MPU6050_ADDR):
        self.addr = addr

        # Calibration offsets
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.accel_offset_z = 0.0
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0

        # Complementary filter state
        self.angle = 0.0

        # Sensitivity scale factors
        # Accel: FS_SEL=0 -> +-2g -> 16384 LSB/g
        self.accel_scale = 16384.0
        # Gyro: FS_SEL=0 -> +-250 deg/s -> 131 LSB/(deg/s)
        self.gyro_scale = 131.0

        self._init_sensor()

    def _write_reg(self, reg, value):
        i2c.write(self.addr, bytes([reg, value]))

    def _read_reg(self, reg, nbytes):
        i2c.write(self.addr, bytes([reg]), repeat=True)
        return i2c.read(self.addr, nbytes)

    def _init_sensor(self):
        """Initialize MPU6050 sensor."""
        # Check WHO_AM_I
        who = self._read_reg(WHO_AM_I, 1)[0]
        if who != 0x68:
            raise OSError("MPU6050 not found (WHO_AM_I=0x{:02X})".format(who))

        # Wake up (clear sleep bit)
        self._write_reg(PWR_MGMT_1, 0x00)
        sleep(100)

        # Sample rate divider: 1kHz / (1+1) = 500Hz
        self._write_reg(SMPLRT_DIV, 0x01)

        # DLPF config: bandwidth ~44Hz (good noise filtering for balancing)
        self._write_reg(CONFIG, 0x03)

        # Gyro config: FS_SEL=0 (+-250 deg/s)
        self._write_reg(GYRO_CONFIG, 0x00)

        # Accel config: AFS_SEL=0 (+-2g)
        self._write_reg(ACCEL_CONFIG, 0x00)

    def _read_raw_data(self):
        """Read all 6 axis raw data in one burst (14 bytes from 0x3B)."""
        data = self._read_reg(ACCEL_XOUT_H, 14)
        # Unpack as 7 signed 16-bit big-endian values
        # accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z
        vals = struct.unpack(">hhhhhhh", data)
        return vals

    def read_accel(self):
        """Read accelerometer data in g."""
        vals = self._read_raw_data()
        ax = vals[0] / self.accel_scale - self.accel_offset_x
        ay = vals[1] / self.accel_scale - self.accel_offset_y
        az = vals[2] / self.accel_scale - self.accel_offset_z
        return ax, ay, az

    def read_gyro(self):
        """Read gyroscope data in deg/s."""
        vals = self._read_raw_data()
        gx = vals[4] / self.gyro_scale - self.gyro_offset_x
        gy = vals[5] / self.gyro_scale - self.gyro_offset_y
        gz = vals[6] / self.gyro_scale - self.gyro_offset_z
        return gx, gy, gz

    def read_all(self):
        """Read accel (g) and gyro (deg/s) in one burst."""
        vals = self._read_raw_data()
        ax = vals[0] / self.accel_scale - self.accel_offset_x
        ay = vals[1] / self.accel_scale - self.accel_offset_y
        az = vals[2] / self.accel_scale - self.accel_offset_z
        gx = vals[4] / self.gyro_scale - self.gyro_offset_x
        gy = vals[5] / self.gyro_scale - self.gyro_offset_y
        gz = vals[6] / self.gyro_scale - self.gyro_offset_z
        return ax, ay, az, gx, gy, gz

    def calibrate(self, samples=200):
        """
        Calibrate sensor offsets. Robot must be upright and still.
        Averages 'samples' readings to determine zero-point offsets.
        """
        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0
        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0

        for _ in range(samples):
            vals = self._read_raw_data()
            sum_ax += vals[0] / self.accel_scale
            sum_ay += vals[1] / self.accel_scale
            sum_az += vals[2] / self.accel_scale
            sum_gx += vals[4] / self.gyro_scale
            sum_gy += vals[5] / self.gyro_scale
            sum_gz += vals[6] / self.gyro_scale
            sleep(2)

        # Gyro offsets: should read 0 when still
        self.gyro_offset_x = sum_gx / samples
        self.gyro_offset_y = sum_gy / samples
        self.gyro_offset_z = sum_gz / samples

        # Accel offsets: X and Y should be 0, Z should be 1g when upright
        self.accel_offset_x = sum_ax / samples
        self.accel_offset_y = sum_ay / samples
        self.accel_offset_z = sum_az / samples - 1.0  # Subtract 1g for Z axis

    def get_accel_angle(self, ax, ay, az):
        """Calculate tilt angle from accelerometer (degrees).
        Returns angle around Y axis (pitch) for front-back balance."""
        # atan2 gives angle in radians, convert to degrees
        return math.atan2(ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

    def get_angle(self, dt):
        """
        Get filtered tilt angle using complementary filter.

        Args:
            dt: Time step in seconds

        Returns:
            Filtered angle in degrees (positive = tilting forward)
        """
        ax, ay, az, gx, gy, gz = self.read_all()

        # Accelerometer angle (noisy but no drift)
        accel_angle = self.get_accel_angle(ax, ay, az)

        # Gyro integration (smooth but drifts)
        # gy is the rotation rate around Y axis
        gyro_delta = gy * dt

        # Complementary filter: trust gyro short-term, accel long-term
        self.angle = 0.98 * (self.angle + gyro_delta) + 0.02 * accel_angle

        return self.angle
