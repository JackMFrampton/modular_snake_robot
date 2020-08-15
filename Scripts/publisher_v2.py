#! /usr/bin/env python
# Author:   Jack M. Frampton
# Language: Python
# Topic:    Snake Motion: Sine-wave

# Import of dependant libraries
# --- --- --- ---
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import time
import numpy as np
import signal
import sys

# Global Variables
# --- --- --- ---
motor_spd = 3.0         #       [rad/s]     (motor speed, set seperately)
mov_up = 0.785          # = 45  [degrees]
home = 0.0              # = 0   [degrees]   (Rest state position)
mov_down = -0.785       # = -45 [degrees]
time_up2down = (abs(mov_up) + abs(mov_down)) / motor_spd    # Time to move between two specified points

# Signal Handler
# --- --- --- ---
# >Capture KeyboardInterrupt
# >Set Snake to home position
# >Quit script appropriately
def signal_handler(sig, frame):
    print("Interrupted")
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
    global snake_head
    global snake_body1
    global snake_body2
    snake_head.publish(0)
    snake_body1.publish(0)
    snake_body2.publish(0)

# Initial Movement Matrix
# --- --- --- ---
# >Used when Snake is put into rest / home position
# >Hence, time delays will be different for initial motion
init_mov = np.array([[   mov_down,      0,              0        ],
                     [   0,             mov_up,         0        ],
                     [   0,             0,              mov_up   ],
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
mov = np.array([[   mov_up,     0,              0           ],
                [   0,          0,              0           ],
                [   0,          mov_down,       0           ],
                [   0,          0,              0           ],
                [   0,          0,              mov_down    ],
                [   0,          0,              0           ],
                [   mov_down,   0,              0           ],
                [   0,          0,              0           ],
                [   0,          mov_up,         0           ],
                [   0,          0,              0           ],
                [   0,          0,              mov_up      ],
                [   0,          0,              0           ],
    ])

# Main Program
# --- --- --- ---
if __name__ == "__main__":
    # start main
    start()
    home()

    time.sleep(3)

    time_delay = time_up2down/6.0

    # Initial Movement Matrix
    # --- --- --- ---
    # Rows
    for i in range(0,3,1):
        # Columns
        for j in range(0,3,1):
            # Pass Filter
            if init_mov[i][j] == 0:
                continue
            else:
                # Snake Head
                if j == 0:
                    snake_head.publish(init_mov[i][j])
                # Snake Body 1
                if j == 1:
                    snake_body1.publish(init_mov[i][j])
                # Snake Body 2
                if j == 2:
                    snake_body2.publish(init_mov[i][j])
        time.sleep(time_delay)

    # Movement Matrix
    # --- --- --- ---
    # >Infinite loop
    # >Can be interrupted (w/ KeyboardInterrupt)
    while True:
        # Rows
        for i in range(0,12,1):
            # Columns
            for j in range(0,3,1):
                # Pass Filter
                if mov[i][j] == 0:
                    continue
                else:
                    # Snake Head
                    if j == 0:
                        snake_head.publish(mov[i][j])
                    # Snake Body 1
                    if j == 1:
                        snake_body1.publish(mov[i][j])
                    # Snake Body 2
                    if j == 2:
                        snake_body2.publish(mov[i][j])
            time.sleep(time_delay)
        signal.signal(signal.SIGINT, signal_handler)
    # end main
