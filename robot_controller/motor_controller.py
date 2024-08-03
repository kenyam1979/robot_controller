import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from robot_interfaces.msg import MotorSpeed
import time
import pigpio
import math
#import threading

MAX_MV = 100
SNSR_FREQ = 30
CTRL_FREQ = 10
INTERVAL = 1/ CTRL_FREQ
WHEEL_DIAMETER = 0.07
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
CAR_WIDTH = 0.125


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

        vel_R = ((count_R - self.prev_count_R)/40/self.interval) * 2*math.pi * WHEEL_DIAMETER / 2.0
        vel_L = ((count_L - self.prev_count_L)/40/self.interval) * 2*math.pi * WHEEL_DIAMETER / 2.0
        ang_R =((count_R - self.prev_count_R)/40/self.interval) * 2*math.pi
        ang_L = ((count_L - self.prev_count_L)/40/self.interval) * 2*math.pi
        self.prev_count_R = count_R 
        self.prev_count_L = count_L
        print(f'Motor speed (m/s) Left={vel_L}, Right={vel_R}, Angular speed (rad/s) Left={ang_L}, Right={ang_R}')
        return (vel_L, vel_R, ang_L, ang_R)

    # def get_motor_angular_speed(self):
    #     global count_R
    #     global count_L

    #     self.prev_count_R = count_R 
    #     self.prev_count_L = count_L
    #     print(f'Motor angular speed (rad/s) Left={ang_L}, Right={ang_R}')
    #     return (ang_L, ang_R)

##### PID controller #####
# PID class calculates manipulating variables by PID
class PID():
    error_P_prev = 0.0
    error_I = 0.0
    interval = INTERVAL
    Kp = 50
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
        print(f'****** target={tar_speed},current={cur_speed} ')
        print(f'****** mv={mv}')
        return mv

##### Deadreckoning #####
# Deadreckoning class calculates current position
class DeadReckoning():
    x = 0
    y = 0
    th = 0
    interval = INTERVAL

    def __init__(self):
        ...

    def get_position(self, a_L, a_R):
        delta_x = (WHEEL_RADIUS / 2.0) * (a_R + a_L) * math.cos(self.th) * self.interval
        delta_y = (WHEEL_RADIUS / 2.0) * (a_R + a_L) * math.sin(self.th) * self.interval
        delta_th = (WHEEL_RADIUS / CAR_WIDTH) * (a_R - a_L) * self.interval 

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        print(f'###### Position x={self.x}, y={self.y}, th={self.th}')


#############################################################
# Power train class to process an entire control of motors
class MotorController (Node):
    
    target_speed_R = 0.0
    target_speed_L = 0.0
    motor_speed_R = 0.0
    motor_speed_L = 0.0

    def __init__(self):
        super().__init__('motor_controller_node')
        self.sub1 = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmdvel_listener_cb,
                SNSR_FREQ)
        
        # self.sub2 = self.create_subscription(
        #         MotorSpeed,
        #         'motor_speed',
        #         self.mtsp_listener_cb,
        #         SNSR_FREQ)

    def cmdvel_listener_cb(self, msg):
        self.get_logger().info(f'linear.x={msg.linear.x} angular.z={msg.angular.z}')
        self.target_speed_R = (msg.linear.x + msg.angular.z/2.0)
        self.target_speed_L = (msg.linear.x - msg.angular.z/2.0)
        
    def drive(self):

        pi = pigpio.pi()
        motor_R = Motor(pi, 18, 17)
        motor_L = Motor(pi, 23, 22)
        pid_R = PID()
        pid_L = PID()
        enc = MotorEncoder(pi, 10, 2)
        dr = DeadReckoning()

        back_flg_R = 1.0
        back_flg_L = 1.0

        rclpy.spin_once(self)


        while rclpy.ok():
            self.motor_speed_L, self.motor_speed_R, a_L, a_R = enc.get_motor_speed()

            if self.target_speed_R < 0.01 and self.target_speed_R > -0.01:
                motor_R.stop()
                pid_R.reset()
            else:
                if self.target_speed_R <0:
                    back_flg_R = -1.0
                mv_R = pid_R.get_manipulating_var(self.target_speed_R, back_flg_R*self.motor_speed_R)
                motor_R.set_manipulating_var(mv_R)
            
            if self.target_speed_L < 0.01 and self.target_speed_L > -0.01:
                motor_L.stop()
                pid_L.reset()
            else:
                if self.target_speed_L <0:
                    back_flg_L = -1.0
                mv_L = pid_L.get_manipulating_var(self.target_speed_L, back_flg_L*self.motor_speed_L)
                motor_L.set_manipulating_var(mv_L)


            dr.get_position(back_flg_L*a_L, back_flg_R*a_R)

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
