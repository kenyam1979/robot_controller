# robot_controller

## Hardware

- 2-wheel differencial drive robot
  - Motor drive: https://www.amazon.co.jp/dp/B083DT2DMV?ref=ppx_yo2ov_dt_b_fed_asin_title
  - Photo encoder: https://www.amazon.co.jp/dp/B084VP1GXS?ref=ppx_yo2ov_dt_b_fed_asin_title
  - Motor, chasis, tires
- Computer
  - Raspberry pi 4B 8GB: https://www.amazon.co.jp/dp/B09G376N9L?ref=ppx_yo2ov_dt_b_fed_asin_title
- Sensors
  - Rplidar A1M8: https://www.amazon.co.jp/gp/product/B07ZD4ML34/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1
- Battery
  - Elecom mobile battery: https://www.amazon.co.jp/dp/B0CTZG7Q8Q?ref=ppx_yo2ov_dt_b_fed_asin_title  

## Software

### motor_controller

#### Topics

- Publish
  - /odom
- Subscribe
  - /cmd_vel


#### Class definition
- Motor
  - This initiates physical motors with pin numbers of raspberry pi
  - This also send values to control physical motors
- MotorEncoder
  - This detects motor speeds by couting photo encoders
  - This has an option to use Kalman filter to smooth values from sensors
- PID
  - This calculates manipulating value based on PID control
- WheelOdom
  - This calculates vehicle position and pose through deadreckoning
- MotorController
  - This is a node to regulate motors with target speed from a teleop controller
  - This publishes odometory information

### Experiments for Kalman filter

This uses pykalman for implementation of Kalman filter. The link below is to experiment its performance using data collected from the vehicle

https://github.com/kenyam1979/robot_controller/blob/main/analysis/Analysis%20for%20Kalman%20filter.ipynb