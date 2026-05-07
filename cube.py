import os
from os.path import isfile
import utils
import numpy as np
from rubik_solver import utils as rubik_utils
<<<<<<< HEAD

state = "start"

if state == "start":
    print("The Rubick's cube solver is in action.")

    # check if the system is calibrated
    if isfile("calibration.txt"):
        print("The system is calibrated.")
    else:
        print("The system is not calibrated. Please calibrate the system before proceeding.")
        exit()
    
    # Start the simulation with CoppeliaSim
    print("Starting the simulation with CoppeliaSim.")

    state = "locate"


if state == "locate":
    print("The cube is being located.")
=======
import yaml

# initial state
state = "start" 


if state == "start":
    
    print("1) System initialization")

    # Check if the system is calibrated
    if isfile("calibration.yaml"):

        print("The system is calibrated.")
        print("Starting the simulation with CoppeliaSim.")
        state = "locate"

    else:
        print("The system is not calibrated. Please calibrate the system before proceeding.")
        


if state == "locate":

    print("2) I'm localizing the cube...")
    
>>>>>>> 728f3cb (aggiunti file yaml)
    cubePose = None

    # Read the image topic from the simulated camera
    print("Reading the image topic from the simulated camera.")

    while cubePose is None:
        img = 0 # get the topic from camera
<<<<<<< HEAD

        cubePose = utils.locate_cube(img)

    print(f"Cube located at: {cubePose}")
    state = "pick"


if state == "pick":
    print("The cube is being picked up.")

    # Move the left robotic arm to the cube's location and pick it up

    # Move the cube in the target position.

    robotPose = [0, 0, 0] # Placeholder for the robot's current pose
    targetPose = [0, 0, 0] # Placeholder for the target pose

    if np.linalg.norm(np.array(robotPose) - np.array(targetPose)) < 0.01:
=======
        cubePose = utils.locate_cube(img)

    print(f"Cube located at: {cubePose}")

    state = "pick"



if state == "pick":

    print("3) I'm picking the cube and i'm moving it to the target position...")
    # Move the left robotic arm to the cube's location and pick it up
    # Move the cube in the target position.

    if isfile("robot_config.json"):
        with open("robot_config.json", "r") as f:
            data_yaml = yaml.safe_load(f)
            robotPose = tuple(data_yaml['robot pose'])
            targetPose = tuple(data_yaml['target pose'])

    if np.linalg.norm(np.array(robotPose) - np.array(targetPose)) < 0.01:
        
>>>>>>> 728f3cb (aggiunti file yaml)
        print("Cube is in the target position.")
        state = "scan"


<<<<<<< HEAD
if state == "scan":
    print("The cube is being scanned.")

    # Rotate the cube and scan it with the camera to determine its configuration

    cubeConfig = utils.scan_cube(img) # Placeholder for the cube's configuration

    if cubeConfig is not None:
        print(f"Cube configuration: {cubeConfig}")
        state = "solve"
    

if state == "solve":
    print("The cube is being solved.")

    # Use a solving algorithm to determine the sequence of moves required to solve the cube
    # Execute the sequence of moves with the robotic arm to solve the cube

    moves = "ok" #rubik_utils.solve(cubeConfig, 'Beginner')

    print(f"Cube solved: {moves}")

    #Handle errors

    state = "execute"


if state == "execute":
    print("Executing the moves to solve the cube.")

    # Execute the sequence of moves with the robotic arm to solve the cube
    for move in moves:
        print(f"Executing move: {move}")
        # Placeholder for executing the move with the robotic arm

    print("Cube solved successfully.")

    state = "check"

if state == "check":
    print("Checking if the cube is solved.")
=======

if state == "scan":

    print("4) Scanning the cube...")

    # Rotate the cube and scan it with the camera to determine its configuration

    cubeConfig = utils.scan_cube(img)

    if cubeConfig is not None:

        print(f"Cube configuration: {cubeConfig}")
        state = "solve"



if state == "solve":

    print("5) Computing the set of moves to solve the cube...")

    # Use a solving algorithm to determine the sequence of moves required to solve the cube
    # Execute the sequence of moves with the robotic arm to solve the cube
    try:
        moves = rubik_utils.solve(cubeConfig, 'Beginner')
        print(f"Cube solved: {moves}")
        state = "execute"
    except Exception as e:
        print("Errore")



if state == "execute":

    print("6) Executing the moves...")

    for move in moves:
        print(f"Executing move: {move}")

    print("Cube solved successfully.")
    state = "check"     



if state == "check":

    print("7) Checking that the cube is solved...")
>>>>>>> 728f3cb (aggiunti file yaml)

    # Check if the cube is solved by scanning it again and comparing the configuration to the solved state

    cubeConfig = utils.scan_cube(img) # Placeholder for the cube's configuration

    if cubeConfig == "solved": # Placeholder for the solved state
        print("Cube is solved.")
    else:
        print("Cube is not solved. Please check the moves executed and try again.")
    
    state = "drop"


<<<<<<< HEAD
if state == "drop":
    print("Dropping the cube.")
=======

if state == "drop":

    print("8) Dropping the cube....")
>>>>>>> 728f3cb (aggiunti file yaml)

    # Move the robotic arm to the drop location and release the cube

    print("Cube dropped successfully.")

    state = "locate"
    
    # Careful, decide how to stop not to enter an infinite loop.
<<<<<<< HEAD
=======














   






    








>>>>>>> 728f3cb (aggiunti file yaml)
