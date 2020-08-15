#! /usr/bin/env python
#

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import time

a = 0.785
b = -0.785
current_pos = None
s = 2
time_delay = 0.35

def start():
    global snake_head
    global snake_body1
    global snake_body2
    snake_head = rospy.Publisher('/J1_controller/command', Float64, queue_size=1)
    snake_body1 = rospy.Publisher('/J2_controller/command', Float64, queue_size=1)
    snake_body2 = rospy.Publisher('/J3_controller/command', Float64, queue_size=1)
    rospy.init_node('Publisher')

#def listen():
#    rospy.Subscriber('/J1_controller/state', JointState, callback_function)

#def callback_function(message):
#    global current_pos
#    current_pos = message.current_pos
#    return current_pos


if __name__ == "__main__":
    start()
    snake_head.publish(0)
    snake_body1.publish(0)
    snake_body2.publish(0)
    time.sleep(3)
    n = 0
    while n < 1:
        snake_head.publish(b)
        time.sleep(time_delay/5)
        snake_body2.publish(a)
        time.sleep(time_delay)
        snake_body1.publish(a)
        time.sleep(-(b/s) - time_delay)
        n = n + 1
    while n < 10:
        snake_head.publish(a)
        time.sleep(time_delay/5)
        snake_body2.publish(b)
        time.sleep(time_delay)
        snake_body1.publish(b)
        time.sleep(-(2*b/s) - time_delay)
        snake_head.publish(b)
        time.sleep(time_delay/5)
        snake_body2.publish(a)
        time.sleep(time_delay)
        snake_body1.publish(a)
        time.sleep(-(2*b/s) - time_delay)
        n = n + 1

    snake_head.publish(0)
    snake_body1.publish(0)
    snake_body2.publish(0)

    #listen()
    #while True:
    #    pub.publish(b)
    #    while True:
    #        listen()
    #        if current_pos <= b + 0.01 and current_pos >= b - 0.01:
    #            break
    #        else:
    #            continue
    #    pub.publish(a)
    #    while True:
    #        listen()
    #        if current_pos <= a + 0.01 and current_pos >= a - 0.01:
    #            break
    #        else:
    #            continue
