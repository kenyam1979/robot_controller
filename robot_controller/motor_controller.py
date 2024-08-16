import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from robot_interfaces.msg import MotorSpeed
import tf_transformations as tft
from tf2_ros.transform_broadcaster import TransformBroadcaster
import time
import pigpio
import math
#import threading

MAX_MV = 100
CTRL_FREQ = 30                      # Hz
INTERVAL = 1/ CTRL_FREQ             # Sec
WHEEL_DIAMETER = 0.07               # Meter
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0 # Meter
CAR_WIDTH = 0.13                    # Meter


##### Motor #####
# Motor class sending PWM signal to physical motor
class Motor():

    def __init__(self, pi, p1, p2):
        self.pi = pi
        self.pin1 = p1
        self.pin2 = p2
        self.pi.set_mode(self.pin1, pigpio.OUTPUT)
        self.pi.set_mode(self.pin2, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pin1, 60)
        self.pi.set_PWM_frequency(self.pin2, 60)
        self.pi.set_PWM_range(self.pin1, MAX_MV)
        self.pi.set_PWM_range(self.pin2, MAX_MV)

    def stop(self):
        self.pi.set_PWM_dutycycle(self.pin1, 0)
        self.pi.set_PWM_dutycycle(self.pin2, 0)

    def set_manipulating_var(self, mv):
        if mv >= 0:
            self.pi.set_PWM_dutycycle(self.pin1, mv)
            self.pi.set_PWM_dutycycle(self.pin2, 0)
        else:
            self.pi.set_PWM_dutycycle(self.pin1, 0)
            self.pi.set_PWM_dutycycle(self.pin2, -mv)
        

##### Motor Encoder #####
count_R = 0
count_L = 0

# Callback function for encoder
def encoder_R_cb(gpio, level, tick):
    global count_R
    count_R += 1

def encoder_L_cb(gpio, level, tick):
    global count_L
    count_L += 1

# Motor photo-encoder
class MotorEncoder():
    interval = INTERVAL
    prev_count_R = 0
    prev_count_L = 0

    def __init__(self, pi, p1, p2):
        self.pi = pi
        self.pin1 = p1
        self.pin2 = p2
        self.pi.set_mode(p1, pigpio.INPUT)
        self.pi.set_pull_up_down(p1, pigpio.PUD_UP)
        self.pi.set_mode(p2, pigpio.INPUT)
        self.pi.set_pull_up_down(p2, pigpio.PUD_UP)
        cb_R = pi.callback(p1, pigpio.EITHER_EDGE, encoder_R_cb)
        cb_L = pi.callback(p2, pigpio.EITHER_EDGE, encoder_L_cb)    

    def get_motor_speed(self):
        global count_R
        global count_L

        vel_R = ((count_R - self.prev_count_R)/40/self.interval) * 2*math.pi * WHEEL_RADIUS
        vel_L = ((count_L - self.prev_count_L)/40/self.interval) * 2*math.pi * WHEEL_RADIUS
        self.prev_count_R = count_R 
        self.prev_count_L = count_L
        # print(f'Motor speed (m/s) Left={vel_L}, Right={vel_R}, Angular speed (rad/s) Left={ang_L}, Right={ang_R}')
        return (vel_L, vel_R)  # Return velocity (m/s)


##### PID controller #####
# PID class calculates manipulating variables by PID
class PID():
    error_P_prev = 0.0
    error_I = 0.0
    interval = INTERVAL
    Kp = 80
    Ki = 100
    Kd = 0.1

    def __init__(self):
        self.interval = INTERVAL

    def reset(self):
        self.error_P_prev = 0.0
        self.error_I = 0.0

    def get_manipulating_var(self, tar_speed, cur_speed) -> float:
        mv = 0.0
        error_P = tar_speed - cur_speed
        self.error_I += error_P * self.interval 
        error_D = (error_P - self.error_P_prev) / self.interval
        
        mv = self.Kp*error_P + self.Ki*self.error_I + self.Kd*error_D

        if mv > MAX_MV:
            mv = MAX_MV
        elif mv < -MAX_MV:
            mv = -MAX_MV

        error_P_prev = error_P
        # print(f'##### <PID> ##### target={tar_speed},current={cur_speed} ')
        # print(f'##### <PID> DDDD mv={mv}')
        return mv


##### Wheel Odom #####
# Deadreckoning class calculates current position
class WheelOdom():

    delta_x = 0.0
    delta_y = 0.0
    delta_th = 0.0
    x = 0.0
    y = 0.0
    th = 0.0
    left = 0.0
    right = 0.0
    velocity = 0.0

    interval = INTERVAL

    def __init__(self):
        ...

    def dead_reckoning(self, vel_L, vel_R):
        self.left = vel_L
        self.right =vel_R

        self.delta_x = (vel_R + vel_L) / 2.0 * math.cos(self.th) 
        self.delta_y = (vel_R + vel_L) / 2.0 * math.sin(self.th)
        self.delta_th = (vel_R - vel_L) / CAR_WIDTH

        self.velocity = (vel_R + vel_L) / 2.0

        self.x += self.delta_x * self.interval
        self.y += self.delta_y * self.interval
        self.th += self.delta_th * self.interval

        # print(f'###### Speed x={self.delta_x}, y={self.delta_y}, th={self.delta_th}')
        # print(f'###### Position x={self.x}, y={self.y}, th={self.th}')



#############################################################
# Power train class to process an entire control of motors
class MotorController (Node):
    
    target_speed_R = 0.0
    target_speed_L = 0.0
    motor_speed_R = 0.0
    motor_speed_L = 0.0


    def __init__(self):
        super().__init__('motor_controller_node')
        self.od = WheelOdom()

        self.sub1 = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmdvel_listener_cb,
                CTRL_FREQ)


        self.tfb = TransformBroadcaster(self)
        self.pub1 = self.create_publisher(
                Odometry, 
                'odom', 
                CTRL_FREQ)

        ## For debugging
        # self.pub2 = self.create_publisher(MotorSpeed, 'motor_speed', CTRL_FREQ)

        self.timer = self.create_timer(INTERVAL, self.odom_publisher_cb)

    def cmdvel_listener_cb(self, msg):
        self.target_speed_R = msg.linear.x + msg.angular.z * CAR_WIDTH / 2.0   # linear.x (m/s) and angular.z (rad/s)
        self.target_speed_L = msg.linear.x - msg.angular.z * CAR_WIDTH / 2.0
        self.get_logger().info(f'linear.x={msg.linear.x} angular.z={msg.angular.z}, target_speed_L={self.target_speed_L}, target_speed_R={self.target_speed_R}')

    def odom_publisher_cb(self):
        now = self.get_clock().now()
        qt_tmp = tft.quaternion_from_euler(0.0, 0.0, self.od.th)
        qt = Quaternion(x=qt_tmp[0], y=qt_tmp[1], z=qt_tmp[2], w=qt_tmp[3])

        # Send tf
        tfs = TransformStamped()
        tfs.header.stamp = now.to_msg()
        tfs.header.frame_id = 'odom'
        tfs.child_frame_id = 'base_link'
        tfs.transform.translation.x = self.od.x
        tfs.transform.translation.y = self.od.y
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation = qt
        self.tfb.sendTransform(tfs)

        # Send odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = self.od.x
        odom.pose.pose.position.y = self.od.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = qt

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.od.velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.od.delta_th
        self.pub1.publish(odom)


        self.get_logger().info(f'x={self.od.x} y={self.od.y}, th={self.od.th}, velocity={self.od.velocity}, angular speed={self.od.delta_th}')

        ## For debugging
        # ms = MotorSpeed()
        # ms.left = self.od.left
        # ms.right = self.od.right
        # self.pub2.publish(ms)


    def drive(self):

        pi = pigpio.pi()
        motor_R = Motor(pi, 18, 17)
        motor_L = Motor(pi, 23, 22)
        pid_R = PID()
        pid_L = PID()
        enc = MotorEncoder(pi, 10, 2)

        rclpy.spin_once(self)


        while rclpy.ok():
            back_flg_R = 1.0
            back_flg_L = 1.0
            self.motor_speed_L, self.motor_speed_R = enc.get_motor_speed()

            if self.target_speed_L < 0.01 and self.target_speed_L > -0.01:
                motor_L.stop()
                pid_L.reset()
            else:
                if self.target_speed_L <0:
                    back_flg_L = -1.0
                mv_L = pid_L.get_manipulating_var(self.target_speed_L, back_flg_L*self.motor_speed_L)
                motor_L.set_manipulating_var(mv_L * 0.91) # Calibration by multipling coeff

            if self.target_speed_R < 0.01 and self.target_speed_R > -0.01:
                motor_R.stop()
                pid_R.reset()
            else:
                if self.target_speed_R <0:
                    back_flg_R = -1.0
                mv_R = pid_R.get_manipulating_var(self.target_speed_R, back_flg_R*self.motor_speed_R)
                motor_R.set_manipulating_var(mv_R) 
            

            self.od.dead_reckoning(back_flg_L*self.motor_speed_L, back_flg_R*self.motor_speed_R)

            rclpy.spin_once(self, timeout_sec=INTERVAL)
            #time.sleep(INTERVAL)

def main(args=None):
    rclpy.init(args=args)

    pt = MotorController()

    pt.drive()
    #rclpy.spin(pt)

    pt.destroy_node()
    rclpy.shutddown()

if __name__ == '__main__':
    main()
