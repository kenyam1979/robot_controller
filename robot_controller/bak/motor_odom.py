import rclpy
from rclpy.node import Node
import math

from robot_interfaces.msg import MotorSpeed

import pigpio

count_R = 0
count_L = 0

class MotorOdom(Node):

    prev_count_R = 0
    prev_count_L = 0
    hz = 30.0
    interval = 1 / hz 

    def __init__(self):
        super().__init__('motor_odom')
        self.publisher = self.create_publisher(MotorSpeed, 'motor_speed', 10)
        self.create_timer(self.interval, self.timer_cb)

    def timer_cb(self):
        global count_R
        global count_L

        msg = MotorSpeed()

        msg.left = ((count_L - self.prev_count_L)/40/self.interval) * 2*math.pi
        msg.right =((count_R - self.prev_count_R)/40/self.interval) * 2*math.pi
        self.prev_count_R = count_R 
        self.prev_count_L = count_L
        self.publisher.publish(msg)
        self.get_logger().info(f'Motor angular (rad/s) msg.left={msg.left}, msg.right={msg.right}')

def encoder_R_cb(gpio, level, tick):
    global count_R
    count_R += 1

def encoder_L_cb(gpio, level, tick):
    global count_L
    count_L += 1

def main(args=None):
    global count_R
    global count_L
    ENC_R = 10
    ENC_L = 2


    pi = pigpio.pi()
    pi.set_mode(ENC_R, pigpio.INPUT)
    pi.set_pull_up_down(ENC_R, pigpio.PUD_UP)
    pi.set_mode(ENC_L, pigpio.INPUT)
    pi.set_pull_up_down(ENC_L, pigpio.PUD_UP)
    cb_R = pi.callback(ENC_R, pigpio.EITHER_EDGE, encoder_R_cb)
    cb_L = pi.callback(ENC_L, pigpio.EITHER_EDGE, encoder_L_cb)

    rclpy.init(args=args)
    mo = MotorOdom()
    rclpy.spin(mo)

    mo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
