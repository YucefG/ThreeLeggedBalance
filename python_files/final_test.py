#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Point
import time 
import numpy as np

MAX_REPET = 5

def create_square_sequence():
    # Create a sequence of stamped points forming a square

    # Square parameters
    center = Point(0.35, 0.08, 0.2) #plane at 0.35m (from robot's base)
    #center = Point(0.25, 0.08, 0.2) #plane at 0.25m
    #center = Point(0.45, 0.08, 0.2) #plane at 0.45m

    width = 0.05                #of tracking square
    height = 0.05

    # Define the four corners of the square
    corners = [
        Point(center.x , center.y - height/2, center.z- width/2),
        Point(center.x , center.y - height/2, center.z+ width/2),
        Point(center.x , center.y + height/2, center.z+ width/2),
        Point(center.x , center.y + height/2, center.z- width/2)
    ]

    # Create the stamped points and add them to the sequence
    sequence = []
    for corner in corners:
        point = PointStamped()
        point.header.frame_id = "odom"  # Modify the frame ID if needed
        point.point = corner
        sequence.append(point)
    return sequence

def marker_array_callback(data):
    '''
    When the first feet position are sensed, wait 2 seconds, and send the next desired position
    Which is one of the 4 corner of the square defined in the plane of point (along y-z plane)
    '''
    # Process the received marker array data, which contains sensed feet positions

    # Example: Print the number of markers received
#   rospy.loginfo("Received {} markers".format(num_markers))

    num_markers = len(data.markers)

    # Store the sensed position
    sensed_position = data.markers[0].pose.position

    # Publish a sequence of stamped points forming a square
    sequence = create_square_sequence()
    global i_corner, num_repetitions, commanded_positions, sensed_positions, seconds


    if( time.time()-seconds > 2):
        print("2 seconds!")
        seconds = time.time()

        #repeat MAX_REPEAT times the square
        if i_corner < len(sequence) and num_repetitions < MAX_REPET:
            commanded_position = sequence[i_corner].point
            sensed_positions.append(sensed_position)
            commanded_positions.append(commanded_position)


            publisher.publish(sequence[i_corner])
            i_corner += 1
            if i_corner == len(sequence):
                num_repetitions += 1
                i_corner = 0

                if num_repetitions == MAX_REPET:
                    # Plot the results
                    plot_results(commanded_positions, sensed_positions)



def plot_results(commanded_positions, sensed_positions):
    # Extract the y and z coordinates of the commanded and sensed positions

    commanded_x = [pos.x for pos in commanded_positions]
    commanded_y = [pos.y for pos in commanded_positions]
    commanded_z = [pos.z for pos in commanded_positions]
    sensed_x = [pos.x for pos in sensed_positions]
    sensed_y = [pos.y for pos in sensed_positions]
    sensed_z = [pos.z for pos in sensed_positions]

    # Save commanded_positions and sensed_positions to text files
    np.savetxt('commanded_positions.txt', np.column_stack((commanded_x, commanded_y, commanded_z)))
    np.savetxt('sensed_positions.txt', np.column_stack((sensed_x, sensed_y, sensed_z)))

    # ...

    print(commanded_positions)
    print(sensed_positions)


    # Plot the commanded and sensed positions in the y-z plane
    plt.figure()
    plt.scatter(commanded_y, commanded_z, color='blue', label='Commanded Positions')
    plt.scatter(sensed_y, sensed_z, color='red', label='Sensed Positions')
    plt.xlabel('Y')
    plt.ylabel('Z')
    plt.title('Commanded vs Sensed Positions')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Calculate the average error in x, y, and z
    errors_x = [abs(cmd.x - sensed.x) for cmd, sensed in zip(commanded_positions, sensed_positions)]
    errors_y = [abs(cmd.y - sensed.y) for cmd, sensed in zip(commanded_positions, sensed_positions)]
    errors_z = [abs(cmd.z - sensed.z) for cmd, sensed in zip(commanded_positions, sensed_positions)]
    avg_error_x = sum(errors_x) / len(errors_x)
    avg_error_y = sum(errors_y) / len(errors_y)
    avg_error_z = sum(errors_z) / len(errors_z)
    abs_avg_error_x = abs(avg_error_x)
    abs_avg_error_y = abs(avg_error_y)
    abs_avg_error_z = abs(avg_error_z)
    rospy.loginfo("Average Error in X: {}".format(avg_error_x))
    rospy.loginfo("Average Error in Y: {}".format(avg_error_y))
    rospy.loginfo("Average Error in Z: {}".format(avg_error_z))
    rospy.loginfo("Absolute Average Error in X: {}".format(abs_avg_error_x))
    rospy.loginfo("Absolute Average Error in Y: {}".format(abs_avg_error_y))
    rospy.loginfo("Absolute Average Error in Z: {}".format(abs_avg_error_z))

def marker_array_listener():
    rospy.init_node('marker_array_listener', anonymous=True)
    rospy.Subscriber('/legged_robot/currentState', MarkerArray, marker_array_callback, queue_size=1)

    # Initialize the corner index, repetition counter, and position lists
    global i_corner, num_repetitions, commanded_positions, sensed_positions, seconds
    seconds = time.time()
    i_corner = 0
    num_repetitions = 0
    commanded_positions = []
    sensed_positions = []

    # Create a publisher
    global publisher
    publisher = rospy.Publisher('/end_effector_pos', PointStamped, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    marker_array_listener()