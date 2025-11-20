#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from example_interfaces.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Path 
from rclpy.clock import Clock
import sys

# --- Navigation and Filtering Constants ---
# Reduced trust in the accelerometer (due to Roll/Pitch error during motion)
COMPLEMENTARY_FILTER_TAU = 0.3 
GRAVITY_CONSTANT = 9.81 

# ZUPT (Zero Velocity Update) Thresholds (for Total Acceleration Data)
ACCEL_THRESHOLD = 0.5  # Allowed extra acceleration when stationary
GYRO_THRESHOLD = 0.02  # Allowed rotation when stationary
PATH_PUBLISH_RATE = 0.5 

# Low-Pass Filter on velocity to reduce accumulated noise
VELOCITY_LPF_ALPHA = 0.8  
# ---

class ImuOdometryNode(Node):
    
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.get_logger().info('IMU Odometry Node Initialized (Total Acceleration Assumption).')

        self.subscription = self.create_subscription(
            Float64MultiArray,  
            'imu_filtered_data', 
            self.imu_filtered_callback,
            10
        )
        self.pose_publisher = self.create_publisher(PoseStamped, 'imu_odometry_pose', 10)
        self.path_publisher = self.create_publisher(Path, 'imu_traversed_path', 10)
        
        # --- Internal State ---
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.path_history = [] 
        
        self.path_publish_timer = self.create_timer(PATH_PUBLISH_RATE, self.publish_path_history)

    
    def imu_filtered_callback(self, msg):
        """ Calculate Odometry and store coordinates in path history """
        if len(msg.data) < 10: return

        accel_body = np.array(msg.data[0:3]) # Total Acceleration (final assumption)
        gyro_rate = np.array(msg.data[3:6])
        dt = msg.data[9]
        if dt <= 0.0: return
            
        # 1. Orientation Estimation
        # Roll/Pitch is calculated using gravity (Total Accel)
        accel_roll = math.atan2(accel_body[1], accel_body[2])
        accel_pitch = math.atan2(-accel_body[0], math.sqrt(accel_body[1]**2 + accel_body[2]**2)) 
        
        # Prediction with gyroscope
        roll_dot = gyro_rate[0] + math.sin(self.roll) * math.tan(self.pitch) * gyro_rate[1] + math.cos(self.roll) * math.tan(self.pitch) * gyro_rate[2]
        pitch_dot = math.cos(self.roll) * gyro_rate[1] - math.sin(self.roll) * gyro_rate[2]
        yaw_dot = math.sin(self.roll) / math.cos(self.pitch) * gyro_rate[1] + math.cos(self.roll) / math.cos(self.pitch) * gyro_rate[2]
        
        gyro_roll = self.roll + roll_dot * dt
        gyro_pitch = self.pitch + pitch_dot * dt
        self.yaw = self.yaw + yaw_dot * dt 
        
        # Combination with Complementary Filter (Reduced trust in accelerometer)
        alpha = COMPLEMENTARY_FILTER_TAU / (COMPLEMENTARY_FILTER_TAU + dt)
        self.roll = alpha * gyro_roll + (1 - alpha) * accel_roll
        self.pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch
        
        
        # 2. Position Estimation
        R = self.create_rotation_matrix()
        accel_world = R @ accel_body # Rotate Total Accel to World Frame
        
        # --- ZUPT (Zero Velocity Update) Logic for TOTAL ACCELERATION ---
        accel_magnitude = np.linalg.norm(accel_body)
        gyro_magnitude = np.linalg.norm(gyro_rate)

        # Calculate predicted velocity before ZUPT
        predicted_velocity = self.velocity + accel_world * dt
        
        # Static condition: If total acceleration magnitude is close to 9.81 and rotation is low.
        if abs(accel_magnitude - GRAVITY_CONSTANT) < ACCEL_THRESHOLD and gyro_magnitude < GYRO_THRESHOLD:
            
            self.get_logger().debug("STATIC DETECTED - ZUPT applied.")
            
            accel_world[:] = 0.0
            self.velocity[:] = 0.0 # Set velocity to zero during stationary state
            
        else:
            # Motion: Remove gravity (since it was Total Accel)
            accel_world[2] = accel_world[2] - GRAVITY_CONSTANT 

            # Velocity integration
            self.velocity += accel_world * dt
            
            # --- Apply Low-Pass Filter on Velocity ---
            self.velocity = VELOCITY_LPF_ALPHA * self.velocity + (1 - VELOCITY_LPF_ALPHA) * predicted_velocity
            
        # Position integration
        self.position += self.velocity * dt
        
        
        # 3. Store and Publish Coordinates
        pose_msg = self.create_pose_message()
        self.path_history.append(pose_msg)
        self.pose_publisher.publish(pose_msg)
        self.get_logger().warn(f"POSITION: X={self.position[0]:.2f}, Y={self.position[1]:.2f}, Z={self.position[2]:.2f}")


    def create_rotation_matrix(self):
        """Create the Body to World rotation matrix (Z-Y-X Convention)"""
        c_r, s_r = math.cos(self.roll), math.sin(self.roll)
        c_p, s_p = math.cos(self.pitch), math.sin(self.pitch)
        c_y, s_y = math.cos(self.yaw), math.sin(self.yaw)

        R = np.array([
            [c_p*c_y, c_y*s_p*s_r - s_y*c_r, c_y*s_p*c_r + s_y*s_r],
            [c_p*s_y, s_y*s_p*s_r + c_y*c_r, s_y*s_p*c_r - c_y*s_r],
            [-s_p,    c_p*s_r,               c_p*c_r]
        ])
        return R

    
    def create_pose_message(self):
        """Create and return a PoseStamped message from the current position and orientation"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Position
        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        
        # Major change: Remove Z-axis from path display in RViz
        # The actual self.position[2] value is preserved in internal calculations, 
        # but is set to zero in the output message for RViz.
        msg.pose.position.z = 0.0 
        
        # Convert Roll/Pitch/Yaw to Quaternion (for Pose message)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return msg

    def publish_path_history(self):
        """Publish the accumulated path history as a nav_msgs/Path message"""
        if not self.path_history:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world' 
        path_msg.poses = self.path_history
        
        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        import numpy as np
    except ImportError:
        print("Error: numpy not installed. Please install with 'pip install numpy'")
        sys.exit(1)
        
    import sys 
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()