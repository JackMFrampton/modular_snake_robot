from multiprocessing import Process

Process(target=serpentine_motion, args=(serpentine_matrix,)).start()
Process(target=sidewinding_motion, args=(sidewinding_matrix,)).start()

leftSW_serpentine_matrix = np.array([   [   serp_pos1,        0,                serp_pos1    ],
                                        [   0,                serp_pos1,        0            ],
                                        [   serp_pos2,        0,                serp_pos2,   ],
                                        [   0,                serp_pos2,        0            ],
    ])
    
leftSW_sidewinding_matrix = np.array([  [   side_pos1,        0,           ],
                                        [   0,                side_pos1,   ],
                                        [   side_pos2,        0,           ],
                                        [   0,                side_pos2,   ],
    ])

def serpentine_motion(mov_array):

    columns = np.size(mov_array,1)
    rows = np.size(mov_array,0)
    
    # Rows
    for i in range(0,rows,1):
        # Columns
        for j in range(0,columns,1):
            # Pass Filter
            if mov_array[i][j] == 0:
                continue
            # Publish movements to motor
            else:
                # Motor 1: Snake Head
                if j == 0:
                    snake_head.publish(mov_array[i][j])
                # Motor 3: Snake Body 2
                if j == 1:
                    snake_body2.publish(mov_array[i][j])
                # Motor 5: Snake Body 4
                if j == 2:
                    snake_body4.publish(mov_array[i][j])
        time.sleep(serp_time_delay)
    
def sidewinding_motion(mov_array):

    columns = np.size(mov_array,1)
    rows = np.size(mov_array,0)
    
    # Rows
    for i in range(0,rows,1):
        # Columns
        for j in range(0,columns,1):
            # Pass Filter
            if mov_array[i][j] == 0:
                continue
            # Publish movements to motor
            else:
                # Motor 2: Snake Body 1
                if j == 0:
                    snake_body1.publish(mov_array[i][j])
                # Motor 4: Snake Body 3
                if j == 1:
                    snake_body3.publish(mov_array[i][j])
        time.sleep(side_time_delay)

# Time from pos_1 -> pos_2
time_up2down = (abs(pos_1) + abs(pos_2)) / motor_spd
# Time delay between each row of the matrix
time_delay = float(time_up2down)/(float(rows)/2)
