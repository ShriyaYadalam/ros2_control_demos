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

class BNO085Publisher(Node):
    def __init__(self):
        super().__init__('bno085_publisher')
        
        # Connection retry parameters
        self.max_retries = 20
        self.retry_delay = 1.0
        self.connection_lost = False
        
        # Initialize IMU with retries
        self.initialize_imu()
        self.imu.begin_calibration()
        
        # Create publisher and timer
        self.publisher = self.create_publisher(Imu, '/demo/imu', 10)
        self.timer = self.create_timer(1.0 / 50, self.publish_imu_data)  # 50Hz
        
        self.get_logger().info("BNO085 Publisher with corrected coordinate frame")
        
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

                self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
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
    
    def correct_quaternion_frame(self, bno_quat):
        """
        Correct the quaternion from BNO085 internal frame to robot frame
        
        Based on your findings:
        - BNO085 X-axis = Your Z-axis  
        - BNO085 Z-axis = Your X-axis
        - BNO085 Y-axis = Your Y-axis
        - Rotation direction needs to be flipped
        
        Args:
            bno_quat: [w, x, y, z] from BNO085
            
        Returns:
            [w, x, y, z] in robot frame
        """
        w, bno_x, bno_y, bno_z = bno_quat
        
        robot_quat = [
            w,          # w component stays the same
            -bno_z,      # robot x = -BNO085 z
            bno_y,      # robot y = BNO085 y  
            -bno_x      # robot z = -BNO085 x (As in IMU, we display in XYZ while ROS takes in ZYX form)
        ]
        
        return robot_quat
    
    def correct_angular_velocity_frame(self, bno_gyro):
        """
        Correct angular velocity from BNO085 frame to robot frame
        
        Args:
            bno_gyro: [x, y, z] from BNO085
            
        Returns:
            [x, y, z] in robot frame
        """
        bno_x, bno_y, bno_z = bno_gyro
        
        # Apply same transformation as quaternion
        robot_gyro = [
            -bno_z,      # robot x = BNO085 z
            bno_y,      # robot y = BNO085 y
            -bno_x      # robot z = -BNO085 x (negative to fix rotation direction)
        ]
        
        return robot_gyro
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians"""
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
    
    def publish_imu_data(self):
        try:
            # Get raw quaternion and gyroscope data from BNO085
            bno_quat = self.imu.quaternion
            bno_gyro = self.imu.gyro
            
            # Skip if no data available
            if bno_quat is None:
                return
            
            # Apply coordinate frame correction
            corrected_quat = self.correct_quaternion_frame(bno_quat)
            
            # Create IMU message
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            # Use corrected quaternion
            msg.orientation.w = corrected_quat[0]
            msg.orientation.x = corrected_quat[1]
            msg.orientation.y = corrected_quat[2]
            msg.orientation.z = corrected_quat[3]
            
            # Convert to Euler angles for debugging
            roll, pitch, yaw = self.quaternion_to_euler(
                corrected_quat[1], corrected_quat[2], corrected_quat[3], corrected_quat[0]
            )
            
            # Convert to degrees for logging
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Log every 10 messages (5Hz at 50Hz publishing rate)
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0
                
            if self.log_counter % 10 == 0:
                self.get_logger().info(f"Raw BNO085 quat: [{bno_quat[0]:.3f}, {bno_quat[1]:.3f}, {bno_quat[2]:.3f}, {bno_quat[3]:.3f}]")
                self.get_logger().info(f"Corrected quat: [{corrected_quat[0]:.3f}, {corrected_quat[1]:.3f}, {corrected_quat[2]:.3f}, {corrected_quat[3]:.3f}]")
                self.get_logger().info(f"Euler angles - Roll: {roll_deg:.1f}°, Pitch: {pitch_deg:.1f}°, Yaw: {yaw_deg:.1f}°")
                self.get_logger().info("-" * 50)
            
            #CHECK IMU ORIENTATION COVARIANCE VALUES
            # Set covariance matrix - only trust yaw for 2-wheel robot
            msg.orientation_covariance = [
                999.0, 0.0, 0.0,    # Roll: high uncertainty (don't trust)
                0.0, 999.0, 0.0,    # Pitch: high uncertainty (don't trust)
                0.0, 0.0, 0.01      # Yaw: low uncertainty (trust this)
            ]
            
            # Handle angular velocity if available
            if bno_gyro is not None:
                corrected_gyro = self.correct_angular_velocity_frame(bno_gyro)
                
                msg.angular_velocity.x = corrected_gyro[0]  # rad/s
                msg.angular_velocity.y = corrected_gyro[1]  # rad/s
                msg.angular_velocity.z = corrected_gyro[2]  # rad/s
                
                #CHECK IMU ANGULAR VELOCITY COVARIANCE VALUES
                # Only trust Z-axis (yaw) angular velocity for 2-wheel robot
                msg.angular_velocity_covariance = [
                    999.0, 0.0, 0.0,    # Roll rate: don't trust
                    0.0, 999.0, 0.0,    # Pitch rate: don't trust
                    0.0, 0.0, 0.02      # Yaw rate: trust this
                ]
                
                if self.log_counter % 10 == 0:
                    self.get_logger().info(f"Corrected gyro: [{corrected_gyro[0]:.3f}, {corrected_gyro[1]:.3f}, {corrected_gyro[2]:.3f}] rad/s")
            else:
                msg.angular_velocity_covariance[0] = -1
            
            # Linear acceleration unavailable
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