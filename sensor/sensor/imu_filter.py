import rclpy
from rclpy.node import Node
import json
import time
from std_msgs.msg import String
from example_interfaces.msg import Float64MultiArray # خروجی فیلتر شده

class ImuFilterNode(Node):
    
    # === پارامترهای فیلتر و بایاس‌ها ===
    # فاکتور فیلتر پایین گذر (کوچکتر = فیلتر قوی‌تر)
    ALPHA_LPF = 0.1 
    
    # بایاس‌های ژیروسکوپ (باید از کالیبراسیون دقیق به دست آیند)
    # **توجه:** این مقادیر صرفاً برای مثال هستند و باید با مقادیر واقعی شما جایگزین شوند.
    GYRO_BIAS_X = 0.002734347138
    GYRO_BIAS_Y = 0.002123482333
    GYRO_BIAS_Z = 0.001512617544

    def __init__(self):
        super().__init__('imu_filter_node')
        self.get_logger().info('IMU Filter Node Initialized. Applying Bias Correction and LPF...')

        # === مشترک شدن در تاپیک ورودی JSON ===
        self.subscription = self.create_subscription(
            String,  
            'imu_sensor_data_json', 
            self.imu_callback,
            10
        )
        
        # === پابلیشر تاپیک خروجی فیلتر شده ===
        # خروجی به صورت Float64MultiArray با ترتیب: Accel(x,y,z), Gyro(x,y,z), Mag(x,y,z)
        self.publisher_ = self.create_publisher(
            Float64MultiArray,  
            'imu_filtered_data',
            10
        )
        
        # === متغیرهای حالت فیلتر (برای LPF) ===
        self.accel_filtered = [0.0, 0.0, 0.0]  # x, y, z
        self.gyro_filtered = [0.0, 0.0, 0.0]   # x, y, z
        self.mag_filtered = [0.0, 0.0, 0.0]    # x, y, z
        
        # برای محاسبه دلتای زمان (dt)
        self.last_time = time.time() 

    def apply_low_pass_filter(self, raw_data, previous_filtered_data):
        """پیاده‌سازی فیلتر پایین گذر مرتبه اول (Exponential Moving Average)"""
        alpha = self.ALPHA_LPF
        filtered_data = []
        for i in range(3):
            # Filtered = alpha * Raw + (1 - alpha) * Previous Filtered
            filtered_value = alpha * raw_data[i] + (1 - alpha) * previous_filtered_data[i]
            filtered_data.append(filtered_value)
        return filtered_data

    def imu_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        try:
            # 1. تجزیه رشته JSON (ساختار Flat)
            data_dict = json.loads(msg.data)
            
            # 2. استخراج داده‌های خام IMU (با استفاده از .get() برای پایداری)
            accel_raw = [
                data_dict.get('accel_x', 0.0), data_dict.get('accel_y', 0.0), data_dict.get('accel_z', 0.0)
            ]
            gyro_raw = [
                data_dict.get('gyro_x', 0.0), data_dict.get('gyro_y', 0.0), data_dict.get('gyro_z', 0.0)
            ]
            mag_raw = [
                data_dict.get('mag_x', 0.0), data_dict.get('mag_y', 0.0), data_dict.get('mag_z', 0.0)
            ]
            
            # زمان اندازه‌گیری (برای نود بعدی لازم است)
            # اگر زمان‌های سنسور متفاوت هستند، زمان gyro یا accel را انتخاب کنید
            time_of_measurement = data_dict.get('accel_time', self.last_time) 

        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON data')
            return
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')
            return

        # 3. اعمال تصحیح بایاس به ژیروسکوپ (مرحله ج)
        gyro_corrected = [
            gyro_raw[0] - self.GYRO_BIAS_X,
            gyro_raw[1] - self.GYRO_BIAS_Y,
            gyro_raw[2] - self.GYRO_BIAS_Z,
        ]

        # 4. اعمال فیلتر پایین گذر (LPF) (مرحله ج)
        self.accel_filtered = self.apply_low_pass_filter(accel_raw, self.accel_filtered)
        self.gyro_filtered = self.apply_low_pass_filter(gyro_corrected, self.gyro_filtered)
        self.mag_filtered = self.apply_low_pass_filter(mag_raw, self.mag_filtered)
        
        # 5. آماده‌سازی پیام خروجی منظم شده (برای نود فیلتر مکمل)
        filtered_msg = Float64MultiArray()
        
        # ترتیب داده‌های خروجی:
        # 1. شتاب‌سنج فیلتر شده
        # 2. ژیروسکوپ فیلتر شده و تصحیح بایاس شده
        # 3. مغناطیس‌سنج فیلتر شده
        # 4. دلتای زمان (dt) برای انتگرال‌گیری دقیق
        filtered_msg.data = (
            self.accel_filtered + 
            self.gyro_filtered + 
            self.mag_filtered + 
            [dt]
        )
        
        # 6. پابلیش داده‌های تمیز شده
        self.publisher_.publish(filtered_msg)
        self.get_logger().debug(f'Published filtered data (dt: {dt:.4f}s)')

def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = ImuFilterNode()
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()