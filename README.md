# robot_controller

## Hardware

- 2-wheel differencial drive robot
  - Motor drive: https://www.amazon.co.jp/dp/B083DT2DMV?ref=ppx_yo2ov_dt_b_fed_asin_title
  - Photo encoder: https://www.amazon.co.jp/dp/B084VP1GXS?ref=ppx_yo2ov_dt_b_fed_asin_title
  - Motor, chasis, tires: 


## motor_controller

### Topics

- Publish
  - /odom
- Subscribe
  - /cmd_vel


### Class definition
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
  - This is a node to regulate motors with target speed from 

## Experiments for Kalman filter

This uses pykalman for implementation of Kalman filter. The link below is to experiment its performance using data collected from the vehicle

https://github.com/kenyam1979/robot_controller/blob/main/analysis/Analysis%20for%20Kalman%20filter.ipynb