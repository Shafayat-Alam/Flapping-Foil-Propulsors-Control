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

Communication
-------------
Publishes: imu_data (sensor_msgs/Imu)
           mag_data (sensor_msgs/MagneticField)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import board
import busio
from adafruit_icm20x import ICM20948


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
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('sample_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        
        # ------------------------------------------------------------------
        # Initialize hardware
        # ------------------------------------------------------------------
        i2c_addr = self.get_parameter('i2c_address').value
        
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = ICM20948(i2c, address=i2c_addr)
            self.get_logger().info(f"ICM20948 initialized at address 0x{i2c_addr:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ICM20948: {e}")
            raise
        
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
        self.get_logger().info(f"Publishing IMU data at {sample_rate} Hz")
    
    def _sample_callback(self):
        """Read IMU sensors and publish data."""
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
            
            # Orientation unknown (would need fusion algorithm)
            imu_msg.orientation_covariance[0] = -1.0
            
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
            self.get_logger().error(f"IMU read error: {e}")


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
