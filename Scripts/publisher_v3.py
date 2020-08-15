#! /usr/bin/env python
# Author:   Jack M. Frampton
# Language: Python
# Topic:    Snake Motion: Sine-wave

# Import of dependant libraries
# --- --- --- ---
import rospy
import time
import numpy as np
import signal
import sys
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

# Global Variables
# --- --- --- ---
# >Python has no static way to assign constant variables
# >Variables below are set before program starts and should not be reassigned
motor_spd = 3.0                                          # [rad/s]                   (motor speed, set seperately)
pos_1 = 0.785                                            # [rad] = 45  [degrees]
home = 0.0                                               # [rad] = 0   [degrees]     (Rest state position)
pos_2 = -0.785                                           # [rad] = -45 [degrees]
time_up2down = (abs(pos_1) + abs(pos_2)) / motor_spd     # [s]                       (Time between two specified points)

# Signal Handler
# --- --- --- ---
# >Capture KeyboardInterrupt
# >Set Snake to home position
# >Quit script appropriately
def signal_handler(sig, frame):
    print("Interrupted")
    home()
    sys.exit(0)

# Shutdown Function
# --- --- --- ---
def shutdown():
    print("Shutting down")
    home()
    sys.exit(0)

# Function to initialise all motors open for manipulation
# --- --- --- ---
# >Creates publisher node
# >All motors can be manipulated from this node
def start():
    global snake_head
    global snake_body1
    global snake_body2
    snake_head = rospy.Publisher('/J1_controller/command', Float64, queue_size=1)
    snake_body1 = rospy.Publisher('/J2_controller/command', Float64, queue_size=1)
    snake_body2 = rospy.Publisher('/J3_controller/command', Float64, queue_size=1)
    rospy.init_node('Publisher')

# Set Snake to home position
# --- --- --- ---
def home():
    snake_head.publish(0)
    snake_body1.publish(0)
    snake_body2.publish(0)

# Initial Movement Matrix
# --- --- --- ---
# >Used when Snake is put into rest / home position
# >Hence, time delays will be different for initial motion
init_mov = np.array([[   pos_2,         0,              0        ],
                     [   0,             pos_1,          0        ],
                     [   0,             0,              pos_1    ],
    ])

# Movement Matrix
# --- --- --- ---
# >Each column represents a different motor
# >e.x. below is a matrix for 3 motors
# >Matrix form allows easy addition of new motors
#  (add new column and adjust rows appropriately)
# >Each row represents a point in the time domain
# >Time delay between each row must be a factor of "time_up2down"
#  (The time taken to move between the two specified points)
# >Increase no. of rows to increase resolution for smoother motion control
mov = np.array([[   pos_1,      0,              0           ],
                [   0,          0,              0           ],
                [   0,          pos_2,          0           ],
                [   0,          0,              0           ],
                [   0,          0,              pos_2       ],
                [   0,          0,              0           ],
                [   pos_2,      0,              0           ],
                [   0,          0,              0           ],
                [   0,          pos_1,          0           ],
                [   0,          0,              0           ],
                [   0,          0,              pos_1       ],
                [   0,          0,              0           ],
    ])

mov_rows = np.size(mov,0)
time_delay = float(time_up2down)/(float(mov_rows)/2)

# Movement Function
# --- --- --- ---
def motion(mov_array):
    columns = np.size(mov_array,1)
    rows = np.size(mov_array,0)
    # Rows
    for i in range(0,rows,1):
        # Columns
        for j in range(0,columns,1):
            # Pass Filter
            if mov_array[i][j] == 0:
                continue
            else:
                # Snake Head
                if j == 0:
                    snake_head.publish(mov_array[i][j])
                # Snake Body 1
                if j == 1:
                    snake_body1.publish(mov_array[i][j])
                # Snake Body 2
                if j == 2:
                    snake_body2.publish(mov_array[i][j])
        time.sleep(time_delay)
    signal.signal(signal.SIGINT, signal_handler)


# Movement Limit Function
# --- --- --- ---
# >Prevent damage to motors
# >Motors should not exceed +1.6 [rad] or go below -1.6 [rad]
def safety_check(a,b):
    safe = True
    if a > 1.6 or a < -1.6:
        print("Position 1 must be between 1.6 [rad] and -1.6 [rad]")
        safe = False
    if b > 1.6 or b < -1.6:
        print("Position 2 must be between 1.6 [rad] and -1.6 [rad]")
        safe = False
    if safe == False:
        shutdown()

# Main Program
# --- --- --- ---
if __name__ == "__main__":
    # start main
    start()
    home()
    safety_check(pos_1,pos_2)

    time.sleep(3)

    # Initial Movement Matrix
    motion(init_mov)

    # Movement Matrix
    # >Infinite loop
    # >Can be interrupted (w/ KeyboardInterrupt)
    while True:
        motion(mov)
    # end main
