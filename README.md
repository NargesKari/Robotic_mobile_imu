# Robotic_mobile_imu

A ROS 2 pipeline that turns a phone's IMU into a live position estimate: an Android sensor-logging app streams raw accelerometer/gyroscope data over HTTP, and three ROS nodes calibrate, filter, and integrate it into a pose and traversed path in RViz.

## What it does

- **`imu_pub`** runs a Flask server (`/data`, port 5000) that receives batched JSON sensor readings from a phone (tested with the Sensor Logger app), queues them, and republishes the latest sample per sensor onto a ROS topic (`imu_sensor_data_json`) at a fixed rate instead of at the phone's raw, bursty send rate.
- **`imu_filter`** auto-calibrates accelerometer/gyroscope bias on startup (collects 25 samples while the device is held still, compares against expected gravity/zero), then low-pass filters the incoming stream and publishes cleaned readings.
- **`imu_odometry`** estimates roll/pitch from gravity and integrates gyro rates for orientation, then double-integrates filtered acceleration into velocity and position, publishing both a live pose and the full traversed path for RViz.
- Also logs raw and filtered readings to CSV/JSONL under `sensor/sensor/data/` for offline inspection.

## Why it's interesting

IMU-only dead reckoning drifts fast, and the project deals with that head-on rather than ignoring it: bias calibration runs automatically before the filter starts trusting any data, and the odometry node applies a low-pass filter on integrated velocity specifically to keep double-integration noise from accumulating into runaway drift. It's also a nice example of bridging a non-ROS data source (a phone's HTTP sensor stream) into a ROS graph — a plain Flask endpoint feeding a queue, decoupled from a separate thread that batches and republishes at a controlled rate, so the ROS side never sees the phone's actual (uneven) transmission timing.

## Tech stack

ROS 2 (`rclpy`), Flask (phone → server bridge), NumPy, `colorama` for console logging output.

## Getting started

Requires a ROS 2 workspace and the `sensor` package built with `colcon`.

```bash
colcon build --packages-select sensor
source install/setup.bash

ros2 run sensor imu_pub        # start the phone-facing HTTP bridge + republisher
ros2 run sensor imu_filter     # calibrate (hold the phone still) and filter
ros2 run sensor imu_odometry   # integrate into pose + path, view in RViz
```

Point a phone sensor-streaming app at `http://<this-machine-ip>:5000/data`. Note that `imu_pub.py` and `imu_server.py` currently hardcode a local output path (`~/Robo/Try3/sensor/sensor`) for saved CSV data — adjust that constant to your own machine before running.
