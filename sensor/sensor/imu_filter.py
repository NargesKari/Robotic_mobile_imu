#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import math
import time
import sys
import numpy as np 
from std_msgs.msg import String
from example_interfaces.msg import Float64MultiArray 

DEFAULT_CUTOFF_FREQ = 5.0 
GRAVITY_CONSTANT = 9.81
CALIBRATION_SAMPLES = 25 
MAX_CALIBRATION_DURATION = 30.0 

class ImuFilterNode(Node):
    
    def __init__(self):
        super().__init__('imu_filter_node')
        self.get_logger().info('IMU Filter Node Initialized. Attempting Auto-Calibration...')

        self.declare_parameter('cutoff_frequency', DEFAULT_CUTOFF_FREQ)
        self.fc = self.get_parameter('cutoff_frequency').value
        self.tau = 1.0 / (2.0 * math.pi * self.fc)

        self.accel_bias = [0.0, 0.0, 0.0] 
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.mag_bias = [0.0, 0.0, 0.0]
        
        self.accel_filtered = [0.0, 0.0, 0.0]
        self.gyro_filtered = [0.0, 0.0, 0.0]
        self.mag_filtered = [0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self._last_printed_time = -1 

        self.auto_calibrate_bias() 
        
        self.subscription = self.create_subscription(
            String,  
            'imu_sensor_data_json', 
            self.imu_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Float64MultiArray,  
            'imu_filtered_data',
            10
        )
        
        self.get_logger().info('Calibration complete. Starting normal operation.')


    def auto_calibrate_bias(self):
        """جمع‌آوری داده‌ها و محاسبه بایاس‌های سنسورها."""
        self.get_logger().warn(f'STARTING CALIBRATION: Keep device static for a maximum of {MAX_CALIBRATION_DURATION:.0f} seconds to collect {CALIBRATION_SAMPLES} samples.')
        
        accel_sum = np.zeros(3)
        gyro_sum = np.zeros(3)
        mag_sum = np.zeros(3)
        sample_count = 0

        def calibration_callback_internal(msg):
            nonlocal sample_count
            if sample_count >= CALIBRATION_SAMPLES:
                return

            try:
                data_dict = json.loads(msg.data)
                
                accel_sum[:] += np.array([data_dict.get('accel_x', 0.0), data_dict.get('accel_y', 0.0), data_dict.get('accel_z', 0.0)])
                gyro_sum[:] += np.array([data_dict.get('gyro_x', 0.0), data_dict.get('gyro_y', 0.0), data_dict.get('gyro_z', 0.0)])
                mag_sum[:] += np.array([data_dict.get('mag_x', 0.0), data_dict.get('mag_y', 0.0), data_dict.get('mag_z', 0.0)])
                
                sample_count += 1
            
            except json.JSONDecodeError:
                self.get_logger().error('JSON error during calibration.')

        temp_sub = self.create_subscription(String, 'imu_sensor_data_json', calibration_callback_internal, 10)
        
        start_time = time.time()
        
        while sample_count < CALIBRATION_SAMPLES and (time.time() - start_time) < MAX_CALIBRATION_DURATION:
            
            elapsed_time = time.time() - start_time
            remaining_time = MAX_CALIBRATION_DURATION - elapsed_time
            
            if remaining_time > 0 and int(remaining_time) != self._last_printed_time:
                print(f'\r[INFO] [imu_filter_node]: Calibration in progress... Collected: {sample_count}/{CALIBRATION_SAMPLES} samples. Time remaining: {int(remaining_time)}s ', end='', file=sys.stdout, flush=True)
                self._last_printed_time = int(remaining_time)

            rclpy.spin_once(self, timeout_sec=0.1) 
            time.sleep(0.01)

        self.destroy_subscription(temp_sub)
        
        print('\r' + ' ' * 100, end='', file=sys.stdout, flush=True) 

        if sample_count < CALIBRATION_SAMPLES:
            self.get_logger().error(f'Failed to receive enough data for calibration. Only {sample_count}/{CALIBRATION_SAMPLES} collected. Using default zero biases.')
            return

        accel_avg = accel_sum / sample_count
        gyro_avg = gyro_sum / sample_count
        mag_avg = mag_sum / sample_count

        self.gyro_bias = gyro_avg.tolist()
        self.mag_bias = mag_avg.tolist()
        
        accel_bias_z = accel_avg[2] #- GRAVITY_CONSTANT 

        self.accel_bias = [accel_avg[0], accel_avg[1], accel_bias_z]
        # ------------------------------------------------------------
        
        self.get_logger().info(f'CALIBRATION SUCCESS: Samples={sample_count}')
        self.get_logger().info(f'Gyro Bias (dps): {self.gyro_bias}')
        self.get_logger().info(f'Accel Bias (m/s^2): {self.accel_bias}')
        self.get_logger().info(f'Mag Bias (uT): {self.mag_bias}')
    
    
    def apply_low_pass_filter(self, raw_data, previous_filtered_data, alpha):
        """اعمال با آلفای محاسبه شده"""
        filtered_data = []
        for i in range(3):
            filtered_value = alpha * raw_data[i] + (1.0 - alpha) * previous_filtered_data[i]
            filtered_data.append(filtered_value)
        return filtered_data

    
    def imu_callback(self, msg):
        """ اعمال بایاس‌های محاسبه شده و فیلتر """
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0.0: return
        
        alpha = dt / (self.tau + dt)

        try:
            data_dict = json.loads(msg.data)
            
            # --- اعمال بایاس‌های محاسبه شده خودکار ---
            accel_raw = [
                data_dict.get('accel_x', 0.0) - self.accel_bias[0], 
                data_dict.get('accel_y', 0.0) - self.accel_bias[1], 
                data_dict.get('accel_z', 0.0) - self.accel_bias[2]
            ]
            gyro_corrected = [
                data_dict.get('gyro_x', 0.0) - self.gyro_bias[0], 
                data_dict.get('gyro_y', 0.0) - self.gyro_bias[1], 
                data_dict.get('gyro_z', 0.0) - self.gyro_bias[2]
            ]
            mag_corrected = [
                data_dict.get('mag_x', 0.0) - self.mag_bias[0], 
                data_dict.get('mag_y', 0.0) - self.mag_bias[1], 
                data_dict.get('mag_z', 0.0) - self.mag_bias[2]
            ]
            # ----------------------------------------

        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode JSON data')
            return
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')
            return

        self.accel_filtered = self.apply_low_pass_filter(accel_raw, self.accel_filtered, alpha)
        self.gyro_filtered = self.apply_low_pass_filter(gyro_corrected, self.gyro_filtered, alpha)
        self.mag_filtered = self.apply_low_pass_filter(mag_corrected, self.mag_filtered, alpha)
        
        filtered_msg = Float64MultiArray()
        
        filtered_msg.data = (
            self.accel_filtered + 
            self.gyro_filtered + 
            self.mag_filtered + 
            [dt]
        )
        
        self.publisher_.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = ImuFilterNode() 
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()