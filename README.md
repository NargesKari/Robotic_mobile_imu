
# 📱 Mobile IMU Sensor Fusion & Odometry

**(پروژه پردازش داده‌های سنسور موبایل و تخمین اودومتری)**

This project implements a complete ROS 2 pipeline to receive, filter, and process raw IMU data from a mobile device to estimate orientation and position (Odometry) and visualize the path in RViz.

این پروژه یک پایپ‌لاین کامل در ROS 2 است که داده‌های خام IMU را از موبایل دریافت کرده، فیلتر می‌کند و با استفاده از الگوریتم‌های تخمین وضعیت، مسیر حرکت (اودومتری) را محاسبه و در RViz نمایش می‌دهد.

-----

# مستندات فنی پروژه
**(مستند فنی پروژه تخمین اودومتری با سنسورهای موبایل)**

این پروژه یک پایپ‌لاین کامل ROS 2 را پیاده‌سازی می‌کند تا داده‌های سنسورهای IMU موبایل را مدیریت، فیلتر و ترکیب کرده و مسیر حرکت را تخمین بزند.

## ۱. 🌐 پل ارتباطی و مدیریت داده (Data Bridge)

این فاز شامل دریافت داده‌های پرنویز سنسورها و تبدیل آن‌ها به یک جریان داده پایدار در ROS 2 بود.

* **اتصال و شبکه:** از یک سرور **Flask** ساده در پایتون برای گوش دادن به درخواست‌های **HTTP POST** از اپلیکیشن موبایل استفاده شد. جزئیات شبکه (IP و Port) به صورت دستی از `ifconfig` گرفته و در اپلیکیشن **Sensor Logger** تنظیم شد.
* **مدیریت نرخ و تاخیر (Queue & Rate Control):**
    * داده‌های ارسالی به صورت بسته‌های **JSON Batches** دریافت شده و وارد یک **صف (Queue)** می‌شوند.
    * برای اعمال یک **تاخیر کنترل‌شده** و ثابت (۱ ثانیه) و جلوگیری از نوسانات نرخ:
        * ماژول پابلیشر با نرخ **۱ هرتز (۱ ثانیه)** فعال می‌شود.
        * در هر سیکل، **آخرین** بسته موجود در صف (حاوی چندین نمونه) برداشته شده و مابقی بسته‌های قدیمی دور ریخته می‌شوند.
        * از بسته انتخاب شده، تنها **آخرین نمونه** از هر سنسور استخراج و روی تاپیک `/imu_sensor_data_json` منتشر می‌شود.
* **پشتیبان‌گیری (Debugging):** برای اطمینان از صحت داده‌های ارسالی، داده‌های JSON منتشر شده به‌طور همزمان در یک فایل **CSV** ذخیره می‌شوند.

***

## ۲. ⚙️ فیلترینگ و کالیبراسیون دقیق (Filtering & Calibration)

نود `imu_filter_node.py` مسئول کالیبره کردن خطاها و آماده‌سازی داده‌ها برای ادغام (Fusion) است.

* **کالیبراسیون خودکار بایاس (Auto-Calibration):**
    * نود در هنگام اجرا، وارد یک فاز **۳۰ ثانیه‌ای** می‌شود و ۲۵ نمونه را در حالت سکون جمع‌آوری می‌کند.
    * این میانگین برای محاسبه **Bias** استفاده می‌شود: Bias شتاب‌سنج با مقایسه با **گرانش ($9.81 m/s^2$)** و Bias ژیروسکوپ با مقایسه با **صفر** محاسبه می‌شود.
* **اعمال فیلتر:**
    * بایاس‌های محاسبه شده از سیگنال‌ها کسر می‌شوند.
    * یک **فیلتر پایین‌گذر دینامیک (Dynamic LPF)** اعمال می‌شود که ضریب صاف‌سازی ($\alpha$) خود را بر اساس **$dt$ واقعی** و فرکانس قطع ثابت تنظیم می‌کند.
* **خروجی:** داده‌های نهایی (شتاب، ژیرو، مگنت و $\Delta t$) روی تاپیک `/imu_filtered_data` منتشر می‌شوند.

***

## ۳. 🧭 چالش اودومتری و راه حل نهایی (Odometry & Visualization)

نود `imu_odometry_node.py` تلاش نهایی برای تخمین موقعیت و جهت‌گیری بود که با چالش‌های فیزیکی بزرگی مواجه شد.

* **تخمین موقعیت (Double Integration):** ما از **فیلتر مکمل (Complementary Filter)** برای جهت‌گیری و **انتگرال‌گیری دوگانه** برای موقعیت استفاده کردیم.
* **علت شکست و تلاش‌ها:**
    * به دلیل نویز شدید سنسورها، خطای **محور Z (ارتفاع)** به صورت **نمایی (Exponential Drift)** رشد کرد و تخمین ۳ بُعدی را از نظر فیزیکی بی‌معنی ساخت (واگرایی منطقی).
    * ما برای کاهش این اثرات، تکنیک **ZUPT (Zero Velocity Update)** را پیاده‌سازی کردیم و آستانه‌های آن را به دقت تنظیم نمودیم. این تلاش‌ها دریفت را کند کردند، اما متوقف نساختند.
* **تصمیم نهایی (2D Visualization):** برای تکمیل تمرین و نمایش مسیر مربع، تصمیم گرفتیم محور **Z را در خروجی نهایی صفر کنیم** تا RViz مسیر را در صفحه $X$-$Y$ نشان دهد و دریفت نجومی $Z$ را نادیده بگیرد.
* **\یکربندی و Launch :**
    * فایل `path.rviz` در پوشه **`config`** کپی و نصب شد تا تنظیمات RViz به صورت خودکار لود شود.
    * یک **Launch File** برای اجرای خودکار نودهای **فیلتر، اودومتری و RViz** ایجاد شد. (نود اول/Flask باید به صورت دستی اجرا شود).
    * عکس تلاش برای مربع کشیدن در پوشه اصلی با نام `path_screenshot` ذخیره شده است.
-----


## 🇺🇸 Project Description (English)

This repository details a complete ROS 2 pipeline designed to manage, filter, and fuse noisy IMU data transmitted from a mobile device to estimate the device's trajectory.

***

## 1. 🌐 Data Acquisition & Rate Control (The Bridge)

This phase established a stable link between the mobile sensor and the ROS 2 environment while controlling the highly irregular input data rate.

* **Network Setup:** We utilized a dedicated Python script running a **Flask** server to listen for asynchronous **HTTP POST** requests containing sensor data. Network configuration (IP and Port) was manually set on the mobile application's **Sensor Logger** based on the host computer's network details.
* **Data Format & Strategy:** Data arrived in irregular **JSON Batches**. To enforce a stable $1\text{ Hz}$ output rate and manage latency:
    * All incoming batches were funneled into a **FIFO Queue**.
    * A ROS 2 Timer, set to fire every **1 second**, processed the queue.
    * The Timer extracted the **last available batch** (the most recent one) and discarded all older packets to prevent accumulating delay.
    * From the selected batch, only the **latest sample** of each sensor (Accel, Gyro, Mag) was extracted and published to the `/imu_sensor_data_json` topic.
* **Debugging:** For verification and integrity checks, the processed JSON data was simultaneously logged to a **CSV file**.

***

## 2. ⚙️ Filtering and Precise Calibration (Filtering Node)

The **`imu_filter_node.py`** was responsible for cleaning the raw sensor stream to prepare the data for the integration stage.

* **Automatic Bias Calibration (Auto-Calibration):**
    * The node enters a **30-second blocking phase** upon startup, requiring the device to remain **strictly static** to collect 25 samples.
    * **Bias Calculation:** The node calculates the average reading during this static phase and uses it as the sensor **Bias**:
        * **Gyro Bias:** Calculated as the measured average (ideal is $0 \text{ rad/s}$).
        * **Accel Bias:** Calculated by comparing the measured $A_z$ with the expected **$9.81 \text{ m/s}^2$** (Gravity), ensuring the bias corrects the sensor's physical offset relative to gravity.
* **Filtering:** The calculated biases are subtracted from the incoming signals. A **Dynamic Low-Pass Filter (LPF)** is then applied. This filter calculates its smoothing coefficient ($\alpha$) based on the current $\Delta t$ and a fixed cutoff frequency, ensuring precise and stable filtering regardless of variations in the input rate.
* **Output:** Cleaned data is published to the `/imu_filtered_data` topic.

***

## 3. 🧭 Odometry Challenge and Final Solution (Odometry Node)

The **`imu_odometry_node.py`** was the core processing engine, attempting to convert filtered rates and accelerations into a stable pose.

* **The Estimation Method:** The node used a **Complementary Filter** for orientation (Roll/Pitch) and **Double Integration** of the corrected acceleration vector for position estimation.
* **The Failure Mode (Drift):** The project confirmed that pure **Dead Reckoning** with mobile IMUs is inherently unstable. Due to small residual noise and unavoidable **Gravity Leakage** into the estimated acceleration vector, the **Z-axis (altitude) drifted exponentially** (logically expected, but visually catastrophic).
* **Mitigation Efforts:** We implemented the robust **ZUPT (Zero Velocity Update)** technique to detect static phases and forcefully set velocity to zero, alongside aggressive tuning of the filter stability ($\tau=0.3$) to minimize coupling errors.
* **Final Compromise (2D Visualization):** To complete the exercise and achieve a stable visualization of the required square path in RViz, the system was configured for $2D$ output:
    * The internal position calculation for $Z$ was maintained (to keep the physics model running).
    * However, the $Z$ component of the final published `PoseStamped` message was manually set to **$0.0$**, projecting the trajectory onto the $X-Y$ plane and suppressing the extreme altitude drift from the display.
* **Visualization Setup:** The final trajectory is published using `nav_msgs/Path.msg`. The custom RViz settings were successfully installed in the **`config`** folder and loaded via the Launch File.

-----

## 🚀 How to Run / نحوه اجرا

برای اجرای کامل پایپ‌لاین، باید سه مرحله زیر را در فضای کاری ROS 2 خود (Workspace) دنبال کنید.

### ۰. آماده‌سازی و بیلد (Setup and Build)

پس از Clone کردن مخزن به پوشه `src` در فضای کاری ROS 2 خود (مانند `~/ros2_ws/src/sensor`)، محیط را آماده و بیلد کنید:

```bash
# رفتن به پوشه اصلی فضای کاری
cd ~/Robo/Try3 

# ۱. نصب پیش‌نیازهای پکیج (مانند numpy)
# اگر قبلا نصب نشده است
rosdep install --from-paths src --ignore-src -r -y

# ۲. بیلد و نصب پکیج
colcon build --packages-select sensor

# ۳. سورس کردن (Source) محیط نصب شده در ترمینال‌های جدید
source install/setup.bash 
```

### ۱. Start the Data Receiver (Flask Server)

این نود باید به صورت دستی در یک ترمینال جداگانه اجرا شود تا سرور HTTP برای دریافت داده‌ها آماده شود.

```bash
# ترمینال ۱: اجرای نود پابلیشر (Flask Server)
ros2 run sensor imu_publisher_node
```

*(اطمینان حاصل کنید که اپلیکیشن موبایل شما داده‌ها را به IP و Port صحیح ارسال می‌کند.)*

### ۲. Launch the Pipeline (Filter, Odometry, RViz)

پس از شروع به کار سرور Flask، کل پایپ‌لاین فیلتر، اودومتری و RViz را در ترمینال دوم اجرا کنید.

```bash
# ترمینال ۲: اجرای Launch File
ros2 launch sensor imu_pipeline.launch.py
```

*توجه: برای کالیبراسیون بایاس، لطفاً گوشی را برای **۳۰ ثانیه** اول کاملاً **ثابت** نگه دارید.

-----


## 📸 Gallery

Below is a screenshot of our attempt to draw a square path using the mobile sensor, visualized in RViz:
![RViz Path Screenshot](https://github.com/NargesKari/Robotic_mobile_imu/blob/main/sensor/rviz_screenshot.png)



