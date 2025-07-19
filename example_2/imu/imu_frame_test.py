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
import math

class IMUFrameTest(Node):
    def __init__(self):
        super().__init__('imu_frame_test')
        
        # Initialize IMU
        self.initialize_imu()
        
        # Create publisher and timer
        self.publisher = self.create_publisher(Imu, '/demo/imu', 10)
        self.timer = self.create_timer(1.0 / 10, self.test_imu_frame)  # 10Hz for easier reading
        
    def initialize_imu(self):
        """Initialize IMU"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = BNO08X_I2C(i2c, address=0x4b)
            time.sleep(0.5)
            
            # Enable rotation vector and gyroscope
            self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.1)
            self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
            time.sleep(0.5)
            
            self.get_logger().info("BNO085 initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            raise
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def test_imu_frame(self):
        try:
            # Get quaternion and gyroscope data
            quat = self.imu.quaternion
            gyro = self.imu.gyro
            
            if quat is None:
                return
            
            # Raw quaternion from BNO085 (format: [w, x, y, z])
            w, x, y, z = quat[0], quat[1], quat[2], quat[3]
            
            # Convert to Euler angles using BNO085's internal frame
            roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)
            
            # Convert to degrees
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Print detailed analysis
            self.get_logger().info("=" * 80)
            self.get_logger().info(f"RAW QUATERNION [w,x,y,z]: [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
            self.get_logger().info(f"COMPUTED EULER ANGLES:")
            self.get_logger().info(f"  Roll  (X-axis): {roll_deg:8.2f}°")
            self.get_logger().info(f"  Pitch (Y-axis): {pitch_deg:8.2f}°")
            self.get_logger().info(f"  Yaw   (Z-axis): {yaw_deg:8.2f}°")
            
            if gyro is not None:
                gyro_x_dps = math.degrees(gyro[0])
                gyro_y_dps = math.degrees(gyro[1])
                gyro_z_dps = math.degrees(gyro[2])
                
                self.get_logger().info(f"ANGULAR VELOCITY (deg/s):")
                self.get_logger().info(f"  X-axis: {gyro_x_dps:8.2f}°/s")
                self.get_logger().info(f"  Y-axis: {gyro_y_dps:8.2f}°/s")
                self.get_logger().info(f"  Z-axis: {gyro_z_dps:8.2f}°/s")
            
            # # Analysis hints
            # self.get_logger().info("ANALYSIS HINTS:")
            # self.get_logger().info("• If rotating around physical Z-axis changes 'Roll' → BNO085 Z-axis = Your X-axis")
            # self.get_logger().info("• If rotating around physical X-axis changes 'Yaw' → BNO085 X-axis = Your Z-axis")
            # self.get_logger().info("• If rotating around physical Y-axis changes 'Pitch' → BNO085 Y-axis = Your Y-axis")
            
            # Create and publish ROS message for RViz visualization
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            # Publish raw quaternion (no transformation yet)
            msg.orientation.w = w
            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
            
            # Set covariance (trust all axes for now during testing)
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            if gyro is not None:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]
                
                msg.angular_velocity_covariance = [
                    0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01
                ]
            else:
                msg.angular_velocity_covariance[0] = -1
            
            msg.linear_acceleration_covariance[0] = -1
            
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading IMU: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IMUFrameTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()