#!/usr/bin/env python3
"""
gazebo_icm20948_interface.py — Hybrid Gazebo/Real ICM20948 Interface
======================================================================
Automatically detects real ICM20948 IMU and uses it if available.

If real IMU detected: publishes real sensor data
If real IMU NOT detected: publishes Gazebo simulated data

Topics
------
Subscribes: /imu (sensor_msgs/Imu) - from Gazebo (fallback)
Publishes:  imu_data (sensor_msgs/Imu) - real or sim data
            mag_data (sensor_msgs/MagneticField) - real or sim data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

# Try to import ICM20948 hardware (optional)
try:
    import board
    import busio
    from adafruit_icm20x import ICM20948
    ICM20948_AVAILABLE = True
except ImportError:
    ICM20948_AVAILABLE = False


class GazeboICM20948Interface(Node):
    """
    Hybrid ICM20948 interface - auto-detects real IMU, falls back to Gazebo sim.
    """
    
    def __init__(self):
        super().__init__('gazebo_icm20948_interface')
        
        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gazebo_imu_topic', '/imu')
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('sample_rate', 100.0)
        
        self.frame_id = self.get_parameter('frame_id').value
        gazebo_topic = self.get_parameter('gazebo_imu_topic').value
        i2c_addr = self.get_parameter('i2c_address').value
        sample_rate = self.get_parameter('sample_rate').value
        
        # Detect real hardware
        self.use_real_imu = False
        self.imu_sensor = None
        
        if ICM20948_AVAILABLE:
            self._detect_real_imu(i2c_addr)
        else:
            self.get_logger().info("ICM20948 library not available - using Gazebo simulation")
        
        # Subscribe to Gazebo IMU (fallback)
        self.gazebo_imu_sub = self.create_subscription(
            Imu,
            gazebo_topic,
            self._gazebo_imu_callback,
            10
        )
        
        # Store latest Gazebo data
        self.latest_gazebo_imu = None
        
        # Publishers (same topics as real IMU)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'mag_data', 10)
        
        # Timer for real IMU sampling (if available)
        if self.use_real_imu:
            timer_period = 1.0 / sample_rate
            self.create_timer(timer_period, self._sample_real_imu)
            self.get_logger().info(
                f"Using REAL ICM20948 IMU at {sample_rate} Hz"
            )
        else:
            self.get_logger().info(
                f"Using Gazebo simulated IMU from {gazebo_topic}"
            )
    
    def _detect_real_imu(self, i2c_addr):
        """Detect if real ICM20948 is connected."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu_sensor = ICM20948(i2c, address=i2c_addr)
            self.use_real_imu = True
            self.get_logger().info(f"Detected REAL ICM20948 at address 0x{i2c_addr:02X}")
        except Exception as e:
            self.get_logger().info(f"No real ICM20948 detected: {e} - using Gazebo simulation")
            self.use_real_imu = False
            self.imu_sensor = None
    
    def _sample_real_imu(self):
        """Read REAL ICM20948 sensor and publish data."""
        if not self.use_real_imu or not self.imu_sensor:
            return
        
        try:
            # Read sensor data
            accel_x, accel_y, accel_z = self.imu_sensor.acceleration  # m/s^2
            gyro_x, gyro_y, gyro_z = self.imu_sensor.gyro              # rad/s
            mag_x, mag_y, mag_z = self.imu_sensor.magnetic             # uT
            
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU message
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
            
            imu_msg.orientation_covariance[0] = -1.0  # Unknown orientation
            
            self.imu_pub.publish(imu_msg)
            
            # Publish magnetometer message
            mag_msg = MagneticField()
            mag_msg.header = Header()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            
            mag_msg.magnetic_field.x = mag_x * 1e-6  # Convert to Tesla
            mag_msg.magnetic_field.y = mag_y * 1e-6
            mag_msg.magnetic_field.z = mag_z * 1e-6
            
            self.mag_pub.publish(mag_msg)
        
        except Exception as e:
            self.get_logger().error(f"Real IMU read error: {e}")
    
    def _gazebo_imu_callback(self, msg: Imu):
        """
        Receive IMU data from Gazebo and republish if no real IMU available.
        """
        self.latest_gazebo_imu = msg
        
        # Only republish if NOT using real IMU
        if not self.use_real_imu:
            # Republish Gazebo IMU data
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            imu_msg.orientation = msg.orientation
            imu_msg.angular_velocity = msg.angular_velocity
            imu_msg.linear_acceleration = msg.linear_acceleration
            
            imu_msg.orientation_covariance = msg.orientation_covariance
            imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            
            self.imu_pub.publish(imu_msg)
            
            # Publish simulated magnetometer
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            
            mag_msg.magnetic_field.x = 0.2 * 1e-6  # Simulated
            mag_msg.magnetic_field.y = 0.4 * 1e-6
            mag_msg.magnetic_field.z = -0.9 * 1e-6
            
            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboICM20948Interface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
