import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from example_interfaces.msg import Float64MultiArray # برای پابلیش خروجی فیلتر شده

class ImuFilterNode(Node):
    
    # === پارامترهای فیلتر ===
    # فاکتور فیلتر پایین گذر (آلفای کوچکتر = فیلتر قوی‌تر)
    ALPHA_LPF = 0.1 
    
    # بایاس‌های ژیروسکوپ (باید از کالیبراسیون اولیه به دست آیند)
    GYRO_BIAS_X = 0.002734347138
    GYRO_BIAS_Y = 0.002123482333
    GYRO_BIAS_Z = 0.001512617544 

    def __init__(self):
        super().__init__('imu_filter_node')
        self.get_logger().info('IMU Filter Node Initialized.')

        # === مشترک شدن در تاپیک ورودی JSON ===
        # استفاده از String زیرا داده‌ها به صورت JSON (رشته) ارسال می‌شوند
        self.subscription = self.create_subscription(
            String,  
            'imu_sensor_data_json', 
            self.imu_callback,
            10
        )
        
        # === پابلیشر تاپیک خروجی فیلتر شده ===
        # خروجی را به صورت آرایه اعداد خام برای نود بعدی ارسال می‌کنیم
        self.publisher_ = self.create_publisher(
            Float64MultiArray,  
            'imu_filtered_data',
            10
        )
        
        # === متغیرهای حالت فیلتر (برای LPF) ===
        self.accel_filtered = [0.0, 0.0, 0.0]  # [x, y, z]
        self.gyro_filtered = [0.0, 0.0, 0.0]   # [x, y, z]

    def apply_low_pass_filter(self, raw_data, previous_filtered_data):
        """پیاده‌سازی فیلتر پایین گذر مرتبه اول (Exponential Moving Average)"""
        alpha = self.ALPHA_LPF
        filtered_data = []
        for i in range(3):
            # LPF: Filtered = alpha * Raw + (1 - alpha) * Previous Filtered
            filtered_value = alpha * raw_data[i] + (1 - alpha) * previous_filtered_data[i]
            filtered_data.append(filtered_value)
        return filtered_data

    def imu_callback(self, msg):
        try:
            # 1. تجزیه رشته JSON
            data_dict = json.loads(msg.data)
            
            # 2. استخراج داده‌های خام مورد نیاز
            accel_raw = [
                data_dict['accel_x'], 
                data_dict['accel_y'], 
                data_dict['accel_z']
            ]
            gyro_raw = [
                data_dict['gyro_x'], 
                data_dict['gyro_y'], 
                data_dict['gyro_z']
            ]

        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON data')
            return
        except KeyError as e:
            self.get_logger().error(f'Missing key in JSON data: {e}')
            return

        # 3. اعمال تصحیح بایاس به ژیروسکوپ
        gyro_corrected = [
            gyro_raw[0] - self.GYRO_BIAS_X,
            gyro_raw[1] - self.GYRO_BIAS_Y,
            gyro_raw[2] - self.GYRO_BIAS_Z,
        ]

        # 4. اعمال فیلتر پایین گذر (LPF)
        # LPF به شتاب‌سنج‌ها (کاهش نویز با فرکانس بالا)
        self.accel_filtered = self.apply_low_pass_filter(accel_raw, self.accel_filtered)
        
        # LPF به ژیروسکوپ‌های تصحیح‌شده (کاهش نویز، قبل از انتگرال‌گیری در مرحله بعد)
        self.gyro_filtered = self.apply_low_pass_filter(gyro_corrected, self.gyro_filtered)
        
        # 5. آماده‌سازی پیام خروجی (برای نود بعدی)
        filtered_msg = Float64MultiArray()
        # ترتیب خروجی: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        filtered_msg.data = self.accel_filtered + self.gyro_filtered
        
        # 6. پابلیش داده‌های فیلتر شده
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = ImuFilterNode()
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()