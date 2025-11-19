from flask import Flask, request
import json
import logging
import time 
import threading
from queue import Queue
from colorama import Fore, Style, init
import os
import sys

# ==============================================================================
# 1. Configuration and Global Variables
# ==============================================================================

init(autoreset=True) 
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

app = Flask(__name__)

# مسیرها را بر اساس سیستم شما تنظیم کنید
PROJECT_DIR = os.path.expanduser('~/Robo/Try3/sensor/sensor') 
DATA_DIR = os.path.join(PROJECT_DIR, "data")                    
OUTPUT_FILENAME = os.path.join(DATA_DIR, "imu_data.csv") # 👈 نام فایل را تغییر دادم
PROCESSING_INTERVAL_SEC = 1.0 
IS_HEADER_WRITTEN = False 
data_queue = Queue() 

# بررسی و ایجاد پوشه
try:
    os.makedirs(DATA_DIR, exist_ok=True)
    logging.info(f"{Fore.BLUE}Data directory confirmed: {DATA_DIR}")
except Exception as e:
    logging.error(f"{Fore.RED}Failed to create data directory {DATA_DIR}: {e}")
    sys.exit(1)

# ==============================================================================
# 2. Flask Route: Data Receiver (بدون تغییر)
# ==============================================================================

@app.route('/data', methods=['POST'])
def receive_data():
    """
    دریافت بچ داده‌ها و افزودن متادیتای بچ و زمان سرور، سپس ارسال به صف.
    """
    try:
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
# 3. Separate Thread for Data Processing and Saving (منطق جدید)
# ----------------------------------------------------------------

def data_processor_thread():
    """
    دریافت بچ‌های انباشته‌شده، انتخاب اولین نمونه از هر سنسور در آخرین بچ، 
    و ذخیره تمام داده‌ها در یک سطر در CSV هر ۱ ثانیه.
    """
    global IS_HEADER_WRITTEN

    # ترتیب ستون‌های ثابت
    FIXED_METADATA_KEYS = [
        'sessionId', 
        'deviceId', 
        'messageId', 
        'server_received_timestamp', 
        # 'time' و 'name' از این لیست حذف شدند زیرا برای هر سنسور مقدار متفاوتی دارند
        'time_of_latest_batch', # یک ستون زمان واحد برای کل سطر ایجاد می‌کنیم
    ]
    
    logging.info(f"{Fore.CYAN}--- Data Processor Started (1 Hz, Single Row Mode) ---")
    
    while True:
        try:
            time.sleep(PROCESSING_INTERVAL_SEC)

            if data_queue.empty():
                continue

            # دریافت آخرین بچ انباشته شده
            latest_sensor_list = None
            server_time_of_latest_batch = None
            while not data_queue.empty():
                current_batch = data_queue.get_nowait()
                latest_sensor_list = current_batch
                # استفاده از زمان دریافت سرور از آخرین رکورد برای نمایش زمان کلی سطر
                if current_batch:
                    server_time_of_latest_batch = current_batch[0].get('server_received_timestamp')

            if latest_sensor_list is None:
                continue

            # --- منطق فیلترینگ جدید: فقط اولین نمونه از هر سنسور ('name') ---
            selected_records_by_name = {}
            
            for record in latest_sensor_list:
                sensor_name = record.get('name')
                
                if sensor_name is None:
                    continue 

                # اگر قبلاً یک نمونه از این سنسور انتخاب نشده باشد، آن را انتخاب کن
                if sensor_name not in selected_records_by_name:
                    selected_records_by_name[sensor_name] = record
            
            records_to_save = selected_records_by_name.values()

            if not records_to_save:
                 logging.info(f"{Fore.YELLOW}No valid records found in the latest sensor list to save.")
                 continue 
            
            # --- جمع‌آوری تمام داده‌ها در یک سطر واحد (Single Row) ---
            final_data_row = {}
            
            # 1. افزودن متادیتای ثابت (از آخرین رکورد انتخابی)
            # از آنجایی که متادیتا (sessionId, deviceId, messageId) در کل بچ ثابت است، 
            # می‌توانیم از اولین رکورد انتخابی استفاده کنیم.
            first_record = next(iter(records_to_save), None)
            if first_record:
                for k in FIXED_METADATA_KEYS:
                    if k == 'time_of_latest_batch':
                         # استفاده از زمان دریافت سرور به عنوان زمان سطر
                         final_data_row[k] = str(server_time_of_latest_batch if server_time_of_latest_batch is not None else first_record.get('server_received_timestamp', ''))
                    else:
                        final_data_row[k] = str(first_record.get(k, ''))
            
            # 2. افزودن تمام مقادیر سنسورها (با پیشوند) و جمع‌آوری کلیدها
            all_sensor_value_keys = set()
            for record in records_to_save:
                sensor_name = record.get('name')
                if not sensor_name: continue
                
                # برای اطمینان از اینکه زمان سنسور هم قابل مشاهده باشد
                final_data_row[f'{sensor_name}_time'] = str(record.get('time', ''))
                all_sensor_value_keys.add(f'{sensor_name}_time') 
                
                values_dict = record.get('values', {})
                for axis, value in values_dict.items():
                    prefixed_key = f"{sensor_name}_{axis}"
                    final_data_row[prefixed_key] = str(value)
                    all_sensor_value_keys.add(prefixed_key) 
            
            # ترکیب کلیدها برای هدر: ثابت + مقادیر سنسورها
            final_keys = FIXED_METADATA_KEYS + sorted(list(all_sensor_value_keys)) 

            # --- ذخیره‌سازی در فایل CSV ---
            with open(OUTPUT_FILENAME, 'a', encoding='utf-8') as f:
                if not IS_HEADER_WRITTEN:
                    f.write(",".join(final_keys) + "\n") 
                    IS_HEADER_WRITTEN = True
                
                # نوشتن سطر کامل
                values = [final_data_row.get(k, '') for k in final_keys] 
                f.write(",".join(values) + "\n")

                logging.info(f"{Fore.GREEN}PROCESSED & SAVED: Single row with {len(records_to_save)} sensors.")

        except Exception as e:
            logging.error(f"{Fore.RED}Error in data processor thread: {e}", exc_info=True)

# ----------------------------------------------------------------
# 4. Application Execution (بدون تغییر)
# ----------------------------------------------------------------

if __name__ == '__main__':
    processor = threading.Thread(target=data_processor_thread, daemon=True)
    processor.start()

    logging.info("----------------------------------------------------------------")
    logging.info(f"{Fore.BLUE}Starting Flask server on http://0.0.0.0:5000/data ...")
    logging.info(f"{Fore.BLUE}Output File: {OUTPUT_FILENAME}")
    logging.info(f"{Fore.BLUE}Processing Rate: {1/PROCESSING_INTERVAL_SEC} Hz (Single Row Saving)")
    logging.info("----------------------------------------------------------------")
    
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)