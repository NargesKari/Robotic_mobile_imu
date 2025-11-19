from flask import Flask, request
import json
import logging
import time 
import threading
from queue import Queue
from colorama import Fore, Style, init
import os
import csv # <--- جدید
import sys # <--- اگرچه قبلا بود، برای دسترسی به متغیرها مفید است

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ... (سایر تنظیمات)
init(autoreset=True) 
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

PROCESSING_INTERVAL_SEC = 1.0 
data_queue = Queue() 

app = Flask(__name__)
# ----------------------------------------------------------------
# ... (تابع receive_data بدون تغییر)
# ----------------------------------------------------------------
@app.route('/data', methods=['POST'])
def receive_data():
    # ... (کد تابع receive_data بدون تغییر)
    # ... (همانند کد اصلی شما)
    try:
        # ... (دریافت و پردازش داده ها)
        data_batch = request.json
        server_time = time.time() 

        if isinstance(data_batch, dict) and 'payload' in data_batch and isinstance(data_batch['payload'], list):
            sensor_list = data_batch['payload']
            
            metadata = {
                'server_received_timestamp': server_time, 
                'messageId': data_batch.get('messageId', ''),
                'sessionId': data_batch.get('sessionId', ''),
                'deviceId': data_batch.get('deviceId', '')
            }
            
            processed_batch = []
            for record in sensor_list:
                new_record = record.copy()
                new_record.update(metadata)
                processed_batch.append(new_record)

        else:
            logging.warning(f"{Fore.YELLOW}Received data does not contain a list under 'payload'. Skipping batch.")
            return 'Payload structure missing or incorrect', 400

        log_message = f"RECEIVED BATCH: MessageID={data_batch.get('messageId', 'N/A')}, Sensors={len(processed_batch)}"
        logging.info(f"{Fore.YELLOW}{log_message}")

        data_queue.put(processed_batch) 
        return 'OK', 200

    except json.JSONDecodeError:
        logging.error(f"{Fore.RED}Invalid JSON format received.")
        return 'Invalid JSON', 400
    except Exception as e:
        logging.error(f"{Fore.RED}Error receiving data: {e}", exc_info=True)
        return 'Internal Server Error', 500
# ----------------------------------------------------------------
# 3. ROS 2 Node: Data Processor and Publisher
# ----------------------------------------------------------------

class DataPublisherNode(Node):

    def __init__(self):
        super().__init__('flask_sensor_publisher_node')
        self.publisher_ = self.create_publisher(String, 'imu_sensor_data_json', 10) 
        timer_period = PROCESSING_INTERVAL_SEC  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"{Fore.CYAN}--- ROS 2 Publisher Started (1 Hz) on topic '/imu_sensor_data_json' ---")
        
        self.csv_file_path = '/home/dynamic/Robo/Try3/sensor/sensor/data/imu_data.csv'
        self.csv_header = [
            'time_of_publish', 'server_received_timestamp', 'sessionId', 'deviceId', 'messageId', 
            'accel_time', 'accel_accuracy', 'accel_x', 'accel_y', 'accel_z',
            'gyro_time', 'gyro_accuracy', 'gyro_x', 'gyro_y', 'gyro_z',
        ]
        self.setup_csv_file()
        self.get_logger().info(f"{Fore.CYAN}--- Logging to CSV file: {self.csv_file_path} ---")

    def setup_csv_file(self):
        """ایجاد دایرکتوری و نوشتن هدر در صورت عدم وجود فایل."""
        try:
            # ایجاد دایرکتوری در صورت عدم وجود
            dir_name = os.path.dirname(self.csv_file_path)
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)
                
            # اگر فایل وجود ندارد، آن را ایجاد کرده و هدر را بنویسید
            if not os.path.exists(self.csv_file_path):
                with open(self.csv_file_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(self.csv_header)
                    
        except Exception as e:
            self.get_logger().error(f"{Fore.RED}Error setting up CSV file: {e}")
            sys.exit(1)


    def write_to_csv(self, records_to_save, json_payload):
        """ذخیره‌سازی تمام داده‌های فیلترشده در یک سطر واحد."""
        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                
                # --- مرحله ۱: آماده‌سازی داده‌ها ---
                # ساخت یک دیکشنری برای ذخیره تمام داده‌های سطر
                row_data = {key: '' for key in self.csv_header}
                
                # افزودن متادیتای اصلی
                row_data['time_of_publish'] = json_payload.get('time_of_publish')
                row_data['server_received_timestamp'] = json_payload.get('server_received_timestamp')
                row_data['sessionId'] = json_payload.get('sessionId')
                row_data['deviceId'] = json_payload.get('deviceId')
                row_data['messageId'] = json_payload.get('messageId')
                
                # افزودن داده‌های سنسور
                for record in records_to_save:
                    sensor_name = record.get('name', '').lower() # مثلاً: accelerometer
                    values = record.get('values', {})
                    
                    # فرض می‌کنیم نام‌های سنسور 'accelerometer' و 'gyroscope' باشند:
                    if 'accel' in sensor_name:
                        prefix = 'accel'
                    elif 'gyro' in sensor_name:
                        prefix = 'gyro'
                    else:
                        continue # سنسور ناشناخته
                    
                    row_data[f'{prefix}_time'] = record.get('time')
                    row_data[f'{prefix}_accuracy'] = record.get('accuracy')
                    row_data[f'{prefix}_x'] = values.get('x')
                    row_data[f'{prefix}_y'] = values.get('y')
                    row_data[f'{prefix}_z'] = values.get('z')

                # --- مرحله ۲: نوشتن سطر نهایی ---
                # مطمئن می‌شویم که مقادیر به ترتیب هدر قرار می‌گیرند
                final_row = [row_data[header] for header in self.csv_header]
                writer.writerow(final_row)
                
                self.get_logger().info(f"{Fore.GREEN}CSV SAVED: 1 row appended with {len(records_to_save)} sensors.")

        except Exception as e:
            self.get_logger().error(f"{Fore.RED}Error writing single row to CSV: {e}")


    def timer_callback(self):
        
        if data_queue.empty():
            return
        # ... (کد استخراج latest_sensor_list و selected_records_by_name بدون تغییر)
        # ... (همانند کد اصلی شما)
        latest_sensor_list = None
        server_time_of_latest_batch = None
        while not data_queue.empty():
            current_batch = data_queue.get_nowait()
            latest_sensor_list = current_batch
            if current_batch:
                server_time_of_latest_batch = current_batch[0].get('server_received_timestamp')

        if latest_sensor_list is None:
            return
        
        selected_records_by_name = {}
        for record in latest_sensor_list:
            sensor_name = record.get('name')
            if sensor_name is None:
                continue 
            if sensor_name not in selected_records_by_name:
                selected_records_by_name[sensor_name] = record
        
        records_to_save = selected_records_by_name.values()

        if not records_to_save:
             self.get_logger().warn(f"{Fore.YELLOW}No valid records found in the latest sensor list to publish.")
             return 
        
        json_payload = {}
        
        # 1. افزودن متادیتای ثابت
        first_record = next(iter(records_to_save), None)
        if first_record:
            json_payload['sessionId'] = first_record.get('sessionId', '')
            json_payload['deviceId'] = first_record.get('deviceId', '')
            json_payload['messageId'] = first_record.get('messageId', '')
            json_payload['server_received_timestamp'] = first_record.get('server_received_timestamp', '')
            json_payload['time_of_publish'] = time.time() # زمان پابلیش شدن نود

        # 2. افزودن مقادیر تمام سنسورهای فیلترشده
        json_payload['sensors'] = {}
        for record in records_to_save:
            sensor_name = record.get('name')
            if not sensor_name: continue
            
            sensor_data = {
                'time': record.get('time', ''),
                'accuracy': record.get('accuracy', ''),
                'values': record.get('values', {})
            }
            json_payload['sensors'][sensor_name] = sensor_data
        
        # --- [جدید] ذخیره در CSV ---
        self.write_to_csv(records_to_save, json_payload)
        
        # --- پابلیش در ROS 2 ---
        msg = String()
        # تبدیل دیکشنری نهایی به رشته JSON
        msg.data = json.dumps(json_payload) 
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"{Fore.GREEN}PUBLISHED: Sensors={len(records_to_save)}, MessageID={json_payload.get('messageId', 'N/A')}")
        self.get_logger().info(f"Published JSON size: {len(msg.data)} bytes")

# ... (توابع run_flask_app، run_ros_node، main و if __name__ == '__main__': بدون تغییر)

def run_flask_app():
    """ تابع اجرای سرور Flask """
    logging.info("----------------------------------------------------------------")
    logging.info(f"{Fore.BLUE}Starting Flask server on http://0.0.0.0:5000/data ...")
    logging.info("----------------------------------------------------------------")
    # debug=False و use_reloader=False ضروری است
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def run_ros_node():
    """ تابع اجرای نود ROS 2 """
    rclpy.init(args=None)
    node = DataPublisherNode()
    try:
        rclpy.spin(node) # نود را بلاک می‌کند و تایمرها را اجرا می‌کند
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()
    run_ros_node()

if __name__ == '__main__':
    main()
