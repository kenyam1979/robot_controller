import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_interfaces.msg import MotorSpeed
import time
import pigpio
import threading

MAX_MV = 100
SNSR_FREQ = 30
CTRL_FREQ = 10
INTERVAL = 1/ CTRL_FREQ
WHEEL_DIAMETER = 0.07

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
        
# PID class calc manipulating variables by PID
class PID():
    error_P_prev = 0.0
    error_I = 0.0
    interval = INTERVAL
    Kp = 20
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


# Power train class to process an entire control of motors
class PowerTrain (Node):
    
    target_speed_R = 0.0
    target_speed_L = 0.0
    motor_speed_R = 0.0
    motor_speed_L = 0.0

    def __init__(self):
        super().__init__('power_train_node')
        self.sub1 = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmdvel_listener_cb,
                SNSR_FREQ)
        
        self.sub2 = self.create_subscription(
                MotorSpeed,
                'motor_speed',
                self.mtsp_listener_cb,
                SNSR_FREQ)

    def cmdvel_listener_cb(self, msg):
        self.get_logger().info(f'linear.x={msg.linear.x} angular.z={msg.angular.z}')
        self.target_speed_R = (msg.linear.x + msg.angular.z) * 1
        self.target_speed_L = (msg.linear.x - msg.angular.z) * 1

    def mtsp_listener_cb(self, msg):
        #self.get_logger().info(f'right={msg.right}, left={msg.left}')
        self.motor_speed_R = msg.right * WHEEL_DIAMETER / 2.0
        self.motor_speed_L = msg.left * WHEEL_DIAMETER / 2.0
        
    def drive(self):

        pi = pigpio.pi()
        motor_R = Motor(pi, 18, 17)
        motor_L = Motor(pi, 23, 22)
        pid_R = PID()
        pid_L = PID()
        rclpy.spin_once(self)


        while rclpy.ok():
            if self.target_speed_R < 0.01 and self.target_speed_R > -0.01:
                motor_R.stop()
                pid_R.reset()
            else:
                back_flg = 1.0
                if self.target_speed_R <0:
                    back_flg = -1.0
                mv_R = pid_R.get_manipulating_var(self.target_speed_R, back_flg*self.motor_speed_R)
                motor_R.set_manipulating_var(mv_R)
            
            if self.target_speed_L < 0.01 and self.target_speed_L > -0.01:
                motor_L.stop()
                pid_L.reset()
            else:
                back_flg = 1.0
                if self.target_speed_L <0:
                    back_flg = -1.0
                mv_L = pid_L.get_manipulating_var(self.target_speed_L, back_flg*self.motor_speed_L)
                motor_L.set_manipulating_var(mv_L)
 
            rclpy.spin_once(self)
            time.sleep(INTERVAL)

def main(args=None):
    rclpy.init(args=args)

    pt = PowerTrain()

    pt.drive()
    #rclpy.spin(pt)

    pt.destroy_node()
    rclpy.shutddown()

if __name__ == '__main__':
    main()
