import os
from os.path import isfile
import numpy as np
import yaml


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
        with open("config.json", "r") as f:
            data_yaml = yaml.safe_load(f)
            cube_config = tuple(data_yaml['config'])
            print(f"La configurazione del cubo è: {cube_config}")

    return cube_config