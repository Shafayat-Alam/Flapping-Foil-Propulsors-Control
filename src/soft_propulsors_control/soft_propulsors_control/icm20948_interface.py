"""
icm20948_imu.py — ICM-20948 9-DOF IMU Interface Node
=====================================================
Adafruit ICM-20948 9-DOF sensor interface node.
https://www.adafruit.com/product/4503

Publishes continuous IMU data (accelerometer, gyroscope, magnetometer) to ROS2.

Hardware:
- 3-axis accelerometer
- 3-axis gyroscope  
- 3-axis magnetometer
- I2C interface (default address 0x69)

Orientation is fused on-board with a complementary filter (gyro integration
corrected toward the accelerometer's gravity tilt for roll/pitch, plus a
tilt-compensated magnetometer heading for yaw) and published in imu_data.orientation
so downstream nodes (the controller's attitude hold) get a usable attitude.

Communication
-------------
Publishes: imu_data (sensor_msgs/Imu)        accel + gyro + fused orientation
           mag_data (sensor_msgs/MagneticField)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import smbus2
from soft_propulsors_control.icm20948_driver import ICM20948


class ICM20948Node(Node):
    """
    ICM-20948 9-DOF IMU node.
    
    Continuously reads and publishes accelerometer, gyroscope, and magnetometer data.
    """
    
    def __init__(self):
        super().__init__('icm20948_imu')
        
        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('i2c_bus', 1)           # I2C bus number (/dev/i2c-N)
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('sample_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        # Complementary-filter weight: fraction of each step trusting the
        # integrated gyro vs. the accelerometer tilt (closer to 1 = smoother but
        # slower to correct drift; closer to 0 = noisier but more accel-locked).
        self.declare_parameter('comp_filter_alpha', 0.98)

        # ------------------------------------------------------------------
        # Initialize hardware
        # ------------------------------------------------------------------
        self._i2c_bus_num = self.get_parameter('i2c_bus').value
        self._i2c_addr = self.get_parameter('i2c_address').value
        self.imu = None
        self._init_warned = False
        self._try_init_imu()   # non-fatal: if the IMU isn't there, keep retrying
        
        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'mag_data', 10)
        
        # ------------------------------------------------------------------
        # Timer for continuous sampling
        # ------------------------------------------------------------------
        sample_rate = self.get_parameter('sample_rate').value
        timer_period = 1.0 / sample_rate
        self.timer = self.create_timer(timer_period, self._sample_callback)

        self.frame_id = self.get_parameter('frame_id').value

        # ------------------------------------------------------------------
        # Orientation filter state (fused roll/pitch/yaw in rad)
        # ------------------------------------------------------------------
        self._alpha = self.get_parameter('comp_filter_alpha').value
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last_t = None
        self._seeded = False

        # Retry hardware init periodically until the IMU appears (hot-plug /
        # power-up), so a missing IMU never crashes the node.
        self._retry_timer = self.create_timer(2.0, self._retry_init_imu)

        self.get_logger().info(f"Publishing IMU data at {sample_rate} Hz")

    def _try_init_imu(self):
        """Attempt to bring up the ICM20948; returns True on success (non-fatal)."""
        try:
            i2c = smbus2.SMBus(self._i2c_bus_num)
            self.imu = ICM20948(i2c, address=self._i2c_addr)
            self.get_logger().info(
                f"ICM20948 initialized on bus {self._i2c_bus_num} "
                f"at address 0x{self._i2c_addr:02X}")
            return True
        except Exception as e:
            self.imu = None
            if not self._init_warned:
                self.get_logger().error(
                    f"ICM20948 not available ({e}) — running WITHOUT the IMU "
                    f"(no orientation data); will keep retrying.")
                self._init_warned = True
            return False

    def _retry_init_imu(self):
        """Periodic retry while the IMU is absent."""
        if self.imu is None:
            self._try_init_imu()

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        """ZYX (roll-pitch-yaw) Euler angles -> (x, y, z, w) quaternion."""
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )

    def _update_orientation(self, accel, gyro, mag):
        """Fuse one accel/gyro/mag sample into roll/pitch/yaw (rad)."""
        ax, ay, az = accel
        gx, gy, gz = gyro
        mx, my, mz = mag

        t = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self._last_t is None else (t - self._last_t)
        self._last_t = t

        # Absolute tilt from the gravity vector (noisy, but driftless)
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if not self._seeded:
            # Seed straight from the accelerometer so we don't ramp in from 0
            self._roll, self._pitch = roll_acc, pitch_acc
            self._seeded = True
        elif 0.0 < dt < 0.5:
            # Complementary filter: integrate gyro, nudge toward accel tilt.
            # A long gap (dt >= 0.5 s) is skipped so a stall can't fling the angle.
            a = self._alpha
            self._roll = a * (self._roll + gx * dt) + (1.0 - a) * roll_acc
            self._pitch = a * (self._pitch + gy * dt) + (1.0 - a) * pitch_acc

        # Tilt-compensated heading from the magnetometer.  Uncalibrated (no
        # hard/soft-iron correction), so treat yaw as a coarse absolute reference.
        cr, sr = math.cos(self._roll), math.sin(self._roll)
        cp, sp = math.cos(self._pitch), math.sin(self._pitch)
        mx_h = mx * cp + mz * sp
        my_h = mx * sr * sp + my * cr - mz * sr * cp
        self._yaw = math.atan2(-my_h, mx_h)

        return self._euler_to_quat(self._roll, self._pitch, self._yaw)

    def _sample_callback(self):
        """Read IMU sensors and publish data."""
        if self.imu is None:
            return   # IMU absent — nothing to publish (retry timer handles reconnect)
        try:
            # Read sensor data
            accel_x, accel_y, accel_z = self.imu.acceleration  # m/s^2
            gyro_x, gyro_y, gyro_z = self.imu.gyro              # rad/s
            mag_x, mag_y, mag_z = self.imu.magnetic             # uT (microtesla)
            
            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU message (accel + gyro)
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id
            
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Fused orientation (complementary filter) — gives the controller a
            # real attitude instead of an identity/unknown quaternion.
            qx, qy, qz, qw = self._update_orientation(
                (accel_x, accel_y, accel_z),
                (gyro_x, gyro_y, gyro_z),
                (mag_x, mag_y, mag_z))
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            # Roll/pitch are accel-anchored (tight); yaw is uncalibrated mag (loose).
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.2

            self.imu_pub.publish(imu_msg)
            
            # Publish magnetometer message
            mag_msg = MagneticField()
            mag_msg.header = Header()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            
            # Convert microtesla to tesla
            mag_msg.magnetic_field.x = mag_x * 1e-6
            mag_msg.magnetic_field.y = mag_y * 1e-6
            mag_msg.magnetic_field.z = mag_z * 1e-6
            
            self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            # A read failure usually means the IMU dropped off the bus — release
            # it so the retry timer re-initialises instead of erroring every tick.
            self.get_logger().error(f"IMU read error ({e}) — dropping IMU, will re-init.")
            self.imu = None
            self._init_warned = False


def main(args=None):
    rclpy.init(args=args)
    node = ICM20948Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
