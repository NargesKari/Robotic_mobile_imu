#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from example_interfaces.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Path 
from rclpy.clock import Clock

# کاهش اعتماد به شتاب‌سنج (برای جلوگیری از خطای Roll/Pitch هنگام حرکت)
COMPLEMENTARY_FILTER_TAU = 0.3 

# ZUPT (Linear Acceleration)
ACCEL_THRESHOLD = 0.5 
GYRO_THRESHOLD = 0.02
PATH_PUBLISH_RATE = 0.5 

VELOCITY_LPF_ALPHA = 0.8  

class ImuOdometryNode(Node):
    
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.get_logger().info('IMU Odometry Node Initialized for LINEAR ACCELERATION input.')

        self.subscription = self.create_subscription(
            Float64MultiArray,  
            'imu_filtered_data', 
            self.imu_filtered_callback,
            10
        )
        self.pose_publisher = self.create_publisher(PoseStamped, 'imu_odometry_pose', 10)
        self.path_publisher = self.create_publisher(Path, 'imu_traversed_path', 10)
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.path_history = [] 
        
        self.path_publish_timer = self.create_timer(PATH_PUBLISH_RATE, self.publish_path_history)

    
    def imu_filtered_callback(self, msg):
        """ محاسبه Odometry و ذخیره مختصات در تاریخچه مسیر """
        if len(msg.data) < 10: return

        accel_body = np.array(msg.data[0:3]) # Linear Acceleration (گرانش حذف شده)
        gyro_rate = np.array(msg.data[3:6])
        dt = msg.data[9]
        if dt <= 0.0: return
            
        # ۱. تخمین جهت (Orientation Estimation) - فیلتر مکمل
        # توجه: این بخش همچنان نیاز به Total Acceleration دارد
        # اما چون Total Accel نداریم، از همین داده‌های Linear استفاده می‌کنیم (یک ضعف ذاتی روش)
        accel_roll = math.atan2(accel_body[1], accel_body[2])
        accel_pitch = math.atan2(-accel_body[0], math.sqrt(accel_body[1]**2 + accel_body[2]**2)) 
        
        roll_dot = gyro_rate[0] + math.sin(self.roll) * math.tan(self.pitch) * gyro_rate[1] + math.cos(self.roll) * math.tan(self.pitch) * gyro_rate[2]
        pitch_dot = math.cos(self.roll) * gyro_rate[1] - math.sin(self.roll) * gyro_rate[2]
        yaw_dot = math.sin(self.roll) / math.cos(self.pitch) * gyro_rate[1] + math.cos(self.roll) / math.cos(self.pitch) * gyro_rate[2]
        
        gyro_roll = self.roll + roll_dot * dt
        gyro_pitch = self.pitch + pitch_dot * dt
        self.yaw = self.yaw + yaw_dot * dt 
        
        alpha = COMPLEMENTARY_FILTER_TAU / (COMPLEMENTARY_FILTER_TAU + dt)
        self.roll = alpha * gyro_roll + (1 - alpha) * accel_roll
        self.pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch
        
        
        # ۲. تخمین موقعیت (Position Estimation)
        R = self.create_rotation_matrix()
        # دوران Linear Accel به World Frame
        accel_world = R @ accel_body 
        
        # --- ZUPT (Zero Velocity Update) Logic for LINEAR ACCELERATION ---
        # توجه: گرانش قبلاً حذف شده است. بزرگی باید نزدیک صفر باشد.
        accel_magnitude = np.linalg.norm(accel_body)
        gyro_magnitude = np.linalg.norm(gyro_rate)

        # محاسبه سرعت پیش‌بینی شده قبل از ZUPT
        predicted_velocity = self.velocity + accel_world * dt
        
        # شرط سکون: اگر بزرگی شتاب خطی و چرخش کم باشد.
        if accel_magnitude < ACCEL_THRESHOLD and gyro_magnitude < GYRO_THRESHOLD:
            
            self.get_logger().debug("STATIC DETECTED - ZUPT applied.")
            
            accel_world[:] = 0.0 # شتاب را در World Frame صفر کن
            self.velocity[:] = 0.0 # صفر کردن سرعت در حالت سکون
            
        else:
            # حرکت: نیازی به حذف گرانش نیست (چون داده ورودی تمیز است)
            pass 

            # انتگرال‌گیری سرعت
            self.velocity += accel_world * dt
            
            # --- اعمال فیلتر پایین‌گذر بر روی سرعت ---
            self.velocity = VELOCITY_LPF_ALPHA * self.velocity + (1 - VELOCITY_LPF_ALPHA) * predicted_velocity
            
        # انتگرال‌گیری موقعیت
        self.position += self.velocity * dt
        
        
        # ۳. ذخیره و انتشار مختصات
        pose_msg = self.create_pose_message()
        self.path_history.append(pose_msg)
        self.pose_publisher.publish(pose_msg)
        self.get_logger().warn(f"POSITION: X={self.position[0]:.2f}, Y={self.position[1]:.2f}, Z={self.position[2]:.2f}")


    def create_rotation_matrix(self):
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
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        msg.pose.position.z = self.position[2]

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