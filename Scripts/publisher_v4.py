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
home_pos = 0.01                                           # [rad] = 0   [degrees]     (Rest state position)
pos_2 = -0.785                                           # [rad] = -45 [degrees]
time_up2down = (abs(pos_1) + abs(pos_2)) / motor_spd     # [s]                       (Time between two specified points)

# Skip Definition
class Skip(Exception): pass

# Signal Handler
# --- --- --- ---
# >Capture KeyboardInterrupt
# >Set Snake to home position
# >Quit script appropriately
def signal_handler(sig, frame):
    print("Interrupted")
    home()
    raise Skip
    #sys.exit(0)

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
    global snake_body3
    global snake_body4
    snake_head = rospy.Publisher('/J1_controller/command', Float64, queue_size=1)
    snake_body1 = rospy.Publisher('/J2_controller/command', Float64, queue_size=1)
    snake_body2 = rospy.Publisher('/J3_controller/command', Float64, queue_size=1)
    snake_body3 = rospy.Publisher('/J4_controller/command', Float64, queue_size=1)
    snake_body4 = rospy.Publisher('/J5_controller/command', Float64, queue_size=1)
    rospy.init_node('Publisher')

# Set Snake to home position
# --- --- --- ---
def home():
    snake_head.publish(0)
    snake_body1.publish(0)
    snake_body2.publish(0)
    snake_body3.publish(0)
    snake_body4.publish(0)

# 3 MOTORS
# Initial Movement Matrix
# --- --- --- ---
# >Used when Snake is put into rest / home position
# >Hence, time delays will be different for initial motion
init_mov = np.array([                   [   pos_2,           0,            0        ],
                                        [   0,             pos_1,          0        ],
                                        [   0,             0,            pos_1      ],
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
mov = np.array([                        [   pos_1,        0,            pos_1       ],
                                        [   0,            0,            0           ],
                                        [   0,            pos_2,        0           ],
                                        [   0,            0,            0           ],
                                        [   0,            0,            0           ],
                                        [   0,            0,            0           ],
                                        [   pos_2,        0,            pos_2       ],
                                        [   0,            0,            0           ],
                                        [   0,            pos_1,        0           ],
                                        [   0,            0,            0           ],
                                        [   0,            0,            0           ],
                                        [   0,            0,            0           ],
    ])

# 5 MOTORS
# Initial Forward Movement Matrix
# --- --- --- ---
FiveMotor_ForwardInit = np.array([      [   pos_2,      0,              0,              0,              pos_1        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_1,          0,              0            ],
    ])

# Forward Movement Matrix
# --- --- --- ---
FiveMotor_ForwardMov = np.array([       [   pos_1,      0,              0,              0,              pos_2        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_2,          0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   pos_2,      0,              0,              0,              pos_1        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_1,          0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
    ])

# Initial Backward Movement Matrix
# --- --- --- ---
FiveMotor_BackInit = np.array([         [   pos_2,      0,              0,              0,              pos_2        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_1,          0,              0            ],
    ])

# Backward Movement Matrix
# --- --- --- ---
FiveMotor_BackMov = np.array([          [   pos_1,      0,              0,              0,              pos_1        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_2,          0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   pos_2,      0,              0,              0,              pos_2        ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              pos_1,          0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
                                        [   0,          0,              0,              0,              0            ],
    ])

# Initial Right Movement Matrix
# --- --- --- ---
FiveMotor_RightInit = np.array([        [   pos_2,       pos_1,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_2,          0           ],
    ])

# Right Movement Matrix
# --- --- --- ---
FiveMotor_RightMov = np.array([         [   pos_1,       pos_2,          0,              0,              pos_2       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_2,          pos_1,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   pos_2,       pos_1,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_2,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
    ])

# Initial Left Movement Matrix
# --- --- --- ---
FiveMotor_LeftInit = np.array([         [   pos_2,       pos_2,         0,               0,              pos_1       ],
                                        [   0,           0,             0,               0,              0           ],
                                        [   0,           0,             pos_1,           pos_1,          0           ],
    ])

# Left Movement Matrix
# --- --- --- ---
FiveMotor_LeftMov = np.array([          [   pos_1,       pos_1,          0,              0,              pos_2       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_2,          pos_2,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   pos_2,       pos_2,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_1,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
    ])

# Initial Turm Right Movement Matrix
# --- --- --- ---
FiveMotor_TurnRightInit = np.array([    [   pos_2,       pos_1,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_1,          0           ],
    ])

# Turn Right Movement Matrix
# --- --- --- ---
FiveMotor_TurnRightMov = np.array([     [   pos_1,       home_pos,       0,              0,              pos_2       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_2,          home_pos,       0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   pos_2,       pos_1,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_1,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
    ])

# Initial Turm Left Movement Matrix
# --- --- --- ---
FiveMotor_TurnLeftInit = np.array([     [   pos_2,       pos_2,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_2,          0           ],
    ])

# Turn Right Movement Matrix
# --- --- --- ---
FiveMotor_TurnLeftMov = np.array([      [   pos_1,       home_pos,       0,              0,              pos_2       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_2,          home_pos,       0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   pos_2,       pos_2,          0,              0,              pos_1       ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              pos_1,          pos_2,          0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
                                        [   0,           0,              0,              0,              0           ],
    ])

mov_rows = np.size(mov,0)
time_delay = float(time_up2down)/(float(mov_rows)/2)

# Movement Function
# --- --- --- ---
def motion(mov_array):
    columns = np.size(mov_array,1)
    rows = np.size(mov_array,0)
    time.sleep(time_delay)
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
                # Snake Body 3
                if j == 3:
                    snake_body3.publish(mov_array[i][j])
                # Snake Body 4
                if j == 4:
                    snake_body4.publish(mov_array[i][j])
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

    time.sleep(2)

    # Initial Forward Movement
    motion(FiveMotor_ForwardInit)

    # Forward Movement
    # >Infinite loop
    # >Can be interrupted (w/ KeyboardInterrupt = Ctrl + C)
    try:
        while True:
            motion(FiveMotor_ForwardMov)
    except Skip:
        pass

    time.sleep(2)

    # Initial Backwards Movement
    motion(FiveMotor_BackInit)

    # Backwards Movement
    try:
        while True:
            motion(FiveMotor_BackMov)
    except Skip:
        pass

    time.sleep(2)

    # Initial Right Movement
    motion(FiveMotor_RightInit)

    # Right Movement
    try:
        while True:
            motion(FiveMotor_RightMov)
    except Skip:
        pass

    time.sleep(2)

    # Initial Left Movement
    motion(FiveMotor_LeftInit)

    # Left Movement
    try:
        while True:
            motion(FiveMotor_LeftMov)
    except Skip:
        pass

    time.sleep(2)

    # Initial Turn Right Movement
    motion(FiveMotor_TurnRightInit)

    # Left Movement
    try:
        while True:
            motion(FiveMotor_TurnRightMov)
    except Skip:
        pass

    time.sleep(2)

    # Initial Turn Left Movement
    motion(FiveMotor_TurnLeftInit)

    # Left Movement
    try:
        while True:
            motion(FiveMotor_TurnLeftMov)
    except Skip:
        sys.exit(0)

    # end main
