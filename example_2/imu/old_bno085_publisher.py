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
    
    def quaternion_to_euler(self, z, y, x, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        Returns angles in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
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

            # msg.orientation.w = quat[0]  # w (real part)
            # msg.orientation.x = quat[1]  # x (i)
            # msg.orientation.y = quat[2]  # y (j)
            # msg.orientation.z = quat[3]  # z (k)

            msg.orientation.w = quat[0]  # w (real part)
            msg.orientation.x = quat[3]  # x (i)
            msg.orientation.y = quat[2]  # y (j)
            msg.orientation.z = quat[1]  # z (k)
            
            # Convert to Euler angles for logging/debugging
            roll, pitch, yaw = self.quaternion_to_euler(quat[1], quat[2], quat[3], quat[0])
            
            # Convert to degrees for easier reading
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Log Euler angles every 10 messages (5Hz logging at 50Hz publishing)
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0
                
            if self.log_counter % 10 == 0:
                self.get_logger().info(f"Euler angles - Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

            
            # try:
            #     sys, gyr, accel, mag = self.imu.detailed_calibration_status
            #     self.get_logger().info(f"Calibration - SYS: {sys}, GYRO: {gyr}, ACCEL: {accel}, MAG: {mag}")
            # except Exception as e:
            #     self.get_logger().warn(f"FAILED cuz : {e}")


            # try:
            #     sys, gyr, accel, mag = self.imu.calibration_status
            #     self.get_logger().info(f"Calibration Status — SYS: {sys}, GYRO: {gyr}, ACCEL: {accel}, MAG: {mag}")
            # except Exception as e:
            #     self.get_logger().warn(f"Failed to read calibration status: {e}")

            # try:
            #     mag_accuracy = self.imu.calibration_status
            #     self.get_logger().info(f"Magnetometer Calibration Accuracy: {mag_accuracy}")
            # except Exception as e:
            #     self.get_logger().warn(f"Failed to read magnetometer calibration status: {e}")


            
            # Orientation covariance
            msg.orientation_covariance = [
                999.0, 0.0, 0.0,
                0.0, 999.0, 0.0,
                0.0, 0.0, 0.01  # YAW
            ]
            
            # msg.orientation_covariance = [
            #     0.01, 0.0, 0.0, #ROLL
            #     0.0, 999.0, 0.0,
            #     0.0, 0.0, 999.0  
            # ]

            # Angular velocity (if available)
            if gyro is not None:
                msg.angular_velocity.x = gyro[0]  # rad/s
                msg.angular_velocity.y = gyro[1]  # rad/s
                msg.angular_velocity.z = gyro[2]  # rad/s
                
                # Angular velocity covariance
                msg.angular_velocity_covariance = [
                    999.0, 0.0, 0.0,
                    0.0, 999.0, 0.0,
                    0.0, 0.0, 0.02  # YAW VELOCITY
                ]

                # msg.angular_velocity_covariance = [
                #     0.01, 0.0, 0.0, #ROLL VELOCITY
                #     0.0, 999.0, 0.0,
                #     0.0, 0.0, 999.0
                # ]

            else:
                msg.angular_velocity_covariance[0] = -1
            
            # Linear acceleration as unavailable
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
                #self.imu.begin_calibration()
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