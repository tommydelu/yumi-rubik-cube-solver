import os
from os.path import isfile
import numpy as np
import yaml
import math


def locate_cube(img: np.ndarray) -> tuple:

    """
    This function contains the logic to locate the cube in the img
    """

    if isfile("config.json"):
        with open("config.json", "r") as f:

            data_yaml = yaml.safe_load(f)
            cube_pose = tuple(data_yaml['cube pose'])
            print(f"La pose del cubo è: {cube_pose}")

    return cube_pose



def scan_cube(img:np.ndarray) -> str:
    
    """
    This function contains the logic to scan the cube in the img
    """

    if isfile("config.yaml"):
        with open("config.yaml", "r") as f:
            data_yaml = yaml.safe_load(f)
            cube_config = tuple(data_yaml['config'])
            print(f"La configurazione del cubo è: {cube_config}")

    return cube_config

def get_colors_seq(img:np.ndarray) -> np.ndarray:

    if isfile("config.yaml"):

        with open("config.yaml","r") as f:

            data_yaml = yaml.safe_load(f)

            bounding_box = tuple(data_yaml['crop grid'])

            print(f"La bounding box del cubo è: {bounding_box}")

            



def get_grasping_pose(cube_pose:np.ndarray, L:float) -> np.ndarray:

    """
    This function returns the pose of the end-effector of the robot before grasping
    """

    # cube pose is in the form [x, y, z, theta, 0 ,0]
    # L is the side lenght of the cube
    # theta is gonna be normalized beetwen 90 - 0 
    # delta is the space we leave beetwen the end-effector and the cube

    x = cube_pose[0]

    y = cube_pose[1]

    z = cube_pose[2]

    theta = cube_pose[3]

    delta = 0.01

    x_r = x - (L/2 + delta)*math.cos(theta)

    y_r = y - (L/2 + delta)*math.sin(theta)

    return np.array([x_r, y_r, z, theta, 0, 0])
