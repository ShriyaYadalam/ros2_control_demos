#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import time

class BNO085Publisher(Node):
    def __init__(self):
        super().__init__('bno085_publisher')
        
        # Connection retry parameters
        self.max_retries = 5
        self.retry_delay = 1.0
        self.connection_lost = False
        
        # Initialize IMU with retries
        self.initialize_imu()
        
        # Create publisher and timer
        self.publisher = self.create_publisher(Imu, '/demo/imu', 10)
        self.timer = self.create_timer(1.0 / 50, self.publish_imu_data)  # 50Hz
        
    def initialize_imu(self):
        """Initialize IMU with retry logic"""
        for attempt in range(self.max_retries):
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.imu = BNO08X_I2C(i2c, address=0x4b)
                
                # Add initialization delay
                time.sleep(0.5)
                
                # Enable rotation vector reporting
                self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
                time.sleep(0.1)
                
                # Enable gyroscope reporting
                self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
                time.sleep(0.5)
                
                self.get_logger().info("BNO085 initialized successfully")
                self.connection_lost = False
                return
                
            except Exception as e:
                self.get_logger().warn(f"IMU initialization attempt {attempt + 1} failed: {e}")
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay)
                else:
                    self.get_logger().error("Failed to initialize BNO085 after all retries")
                    raise RuntimeError("BNO085 IMU initialization failed")
        
    def publish_imu_data(self):
        try:
            # Get quaternion and gyroscope data
            quat = self.imu.quaternion
            gyro = self.imu.gyro
            
            # Skip if no data available
            if quat is None:
                return
            
            # Create IMU message
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            # Orientation (quaternion) - Adafruit format is [w, x, y, z]
            msg.orientation.w = quat[0]  # w (real part)
            msg.orientation.x = quat[1]  # x (i)
            msg.orientation.y = quat[2]  # y (j)
            msg.orientation.z = quat[3]  # z (k)
            
            # Orientation covariance (tune these values based on your sensor accuracy)
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            # Angular velocity (if available)
            if gyro is not None:
                msg.angular_velocity.x = gyro[0]  # rad/s
                msg.angular_velocity.y = gyro[1]  # rad/s
                msg.angular_velocity.z = gyro[2]  # rad/s
                
                # Angular velocity covariance
                msg.angular_velocity_covariance = [
                    0.02, 0.0, 0.0,
                    0.0, 0.02, 0.0,
                    0.0, 0.0, 0.02
                ]
            else:
                msg.angular_velocity_covariance[0] = -1
            
            # Linear acceleration as unavailable (or add accelerometer if needed)
            msg.linear_acceleration_covariance[0] = -1
            
            # Publish the message
            self.publisher.publish(msg)
            
            # Reset connection lost flag if we successfully got data
            if self.connection_lost:
                self.get_logger().info("IMU connection recovered")
                self.connection_lost = False
            
        except Exception as e:
            # Handle connection errors gracefully
            if not self.connection_lost:
                self.get_logger().warn(f"IMU connection lost: {e}")
                self.connection_lost = True
            
            # Try to reinitialize if connection is lost
            try:
                self.get_logger().info("Attempting to reinitialize IMU...")
                self.initialize_imu()
            except Exception as reinit_error:
                self.get_logger().error(f"Failed to reinitialize IMU: {reinit_error}")
                # Continue trying in next cycle

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BNO085Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()