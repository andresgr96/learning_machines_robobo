from data_files import FIGRURES_DIR
import numpy as np
import math
from scipy import stats
import time

from robobo_interface import (
    IRobobo,
    Emotion,
    LedId,
    LedColor,
    SoundEmotion,
    SimulationRobobo,
    HardwareRobobo,
)


# {'BackL': -6.4, 'BackR': -6.4, 'FrontL': -52.3, 'FrontR': -52.3, 'FrontC': -5.8, 'FrontRR': -5.9, 'BackC': -57.8, 'FrontLL': -5.9}
# {'x': 0.0, 'y': 0.0, 'z': 0.0}

{'BackL': 18724.6, 'BackR': 19794.6, 'FrontL': 23174.7, 'FrontR': 18005.7, 'FrontC': 18841.2, 'FrontRR': 17139.1, 'BackC': 17270.2, 'FrontLL': 14951.1}


def read_irs_sensors(rob, num_reads=7):
    joint_list = ["BackL", "BackR", "FrontL", "FrontR", "FrontC", "FrontRR", "BackC", "FrontLL"]
    no_obstacle_sens_values = [6.434948026696321, 6.4375698872759655, 52.26984940735039, 52.270314744820546, 5.845623601383301, 5.890924916422574, 57.76850943075616, 5.925058770384208]

    readings = {joint: [] for joint in joint_list}
    for _ in range(num_reads):
        temp = rob.read_irs()
        temp2 = []
        for i, val in enumerate(temp):
            if val != math.inf:          
                temp2.append(abs(temp[i]))
            else:
                temp2.append(abs(no_obstacle_sens_values[i]))
        irs_data = np.round(np.array(temp2) - np.array(no_obstacle_sens_values), decimals=1)
        for joint, value in zip(joint_list, irs_data):
            readings[joint].append(value)
    sensor_modes = {joint: stats.mode(values)[0][0] for joint, values in readings.items()}
    print(sensor_modes)
    return sensor_modes



def read_accel_sensors(rob, num_reads=7):
    readings = {"x": [], "y": [], "z": []}

    for _ in range(num_reads):
        temp = rob.read_accel()
        readings["x"].append(temp.x)
        readings["y"].append(temp.y)
        readings["z"].append(temp.z)

    sensor_modes = {axis: round(stats.mode(values)[0][0], 1) for axis, values in readings.items()}
    print(sensor_modes)
    return sensor_modes



def move_forward(rob, speed, duration):
    """
    Move the robot forward.
    """
    rob.move_blocking(left_speed=speed, right_speed=speed, millis=duration)

def move_backward(rob, speed, duration):
    """
    Move the robot backward.
    """
    rob.move_blocking(left_speed=-speed, right_speed=-speed, millis=duration)

def turn_left(rob, speed, duration):
    """
    Turn the robot left.
    """
    rob.move_blocking(left_speed=-speed, right_speed=speed, millis=duration)

def turn_right(rob, speed, duration):
    """
    Turn the robot right.
    """
    rob.move_blocking(left_speed=speed, right_speed=-speed, millis=duration)




def avoid_obstacle(rob: IRobobo):
    """
    The robot goes straight until it senses an object getting near, then steers to the right.

    Args:
        rob: The IRobobo object representing the robot.

    Returns:
        None
    """

    threshold = 7.5 
    speed_threshold = 35 
    avoid_duration = 500 
    count = 0

    while count < 50:
        sensor_dict = read_irs_sensors(rob)
        print(sensor_dict)

        if (sensor_dict["FrontC"] > threshold or
            sensor_dict["FrontR"] > threshold or
            sensor_dict["FrontL"] > threshold):
            # Check which side has more space and turn accordingly
            if sensor_dict["FrontR"] > sensor_dict["FrontL"]:
                turn_left(rob, speed_threshold, avoid_duration)
            else:
                turn_right(rob, speed_threshold, avoid_duration)
        else:
            # Move forward
            move_forward(rob, speed_threshold, avoid_duration)

        count += 1


        

def touch_wall_backup(rob: IRobobo):
    """
    The robot goes straight until it touches a wall, then goes backward.
    """
    threshold = 250  # Adjust this value based on your sensor noise and desired sensitivity
    speed_threshold = 50
    avoid_duration = 1000  # Adjust this value for turning smoothness
    count = 0
    wall_touched = False  # Flag to indicate if the wall has been touched

    while count < 20:
        sensor_dict = read_irs_sensors(rob)
        accel_dict = read_accel_sensors(rob)
        time.sleep(2)

        if (sensor_dict["FrontC"] > threshold or
            sensor_dict["FrontR"] > threshold or
            sensor_dict["FrontL"] > threshold):

            wall_touched = True  # Set the flag to True if the wall is touched

        if wall_touched:
            move_backward(rob, speed_threshold, avoid_duration)
        else:
            move_forward(rob, speed_threshold, avoid_duration)

        count += 1



        
    