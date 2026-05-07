import os
from os.path import isfile
import utils
import numpy as np
from rubik_solver import utils as rubik_utils
import yaml



# initial state
state = "start" 

while state != "stop":


    if state == "start":
        
        print("1) System initialization")

        # Check if the system is calibrated
        if isfile("calibration.yaml"):

            print("The system is calibrated.")
            print("Starting the simulation with CoppeliaSim.")

            #TODO load the calibration file 



            state = "locate"

        else:
            print("The system is not calibrated. Please calibrate the system before proceeding.")
            


    if state == "locate":

        print("2) I'm localizing the cube...")
        
        cubePose = None

        # Read the image topic from the simulated camera
        print("Reading the image topic from the simulated camera.")

        while cubePose is None:

            img = 0 # get the topic from camera

            #TODO write the function to estimate 6D of the cube

            cubePose = utils.locate_cube(img)

        print(f"Cube located at: {cubePose}")

        state = "pick"




    if state == "pick":

        print("3) I'm picking the cube and i'm moving it to the target position...")

        # Move the left robotic arm to the cube's location and pick it up
        grasping_pose = utils.get_grasping_pose(cubePose, L = 0.07)

        #TODO comunicate to ros to move the end-effector to the grasping position and close the gripper 

        # Move the cube in the target position.

        if isfile("robot_config.json"):
            with open("robot_config.json", "r") as f:
                data_yaml = yaml.safe_load(f)
                robotPose = tuple(data_yaml['robot pose'])
                targetPose = tuple(data_yaml['target pose'])

        #TODO comunicate to ros to move the end-effector to the target pose

        if np.linalg.norm(np.array(robotPose) - np.array(targetPose)) < 0.01:
            
            print("Cube picked and moved to the target position successfully.")
            state = "scan"


    if state == "execute":
        print("Executing the moves to solve the cube.")
        moves = None

        # Execute the sequence of moves with the robotic arm to solve the cube
        for move in moves:
            print(f"Executing move: {move}")
            # Placeholder for executing the move with the robotic arm

        print("Cube solved successfully.")

        state = "check"



    if state == "check":
        print("Checking if the cube is solved.")



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

        # Check if the cube is solved by scanning it again and comparing the configuration to the solved state

        cubeConfig = utils.scan_cube(img) # Placeholder for the cube's configuration

        if cubeConfig == "solved": # Placeholder for the solved state
            print("Cube is solved.")
        else:
            print("Cube is not solved. Please check the moves executed and try again.")
        
        state = "drop"


    if state == "drop":

        print("8) Dropping the cube....")

        # Move the robotic arm to the drop location and release the cube

        print("Cube dropped successfully.")

        state = "locate"
        
        # Careful, decide how to stop not to enter an infinite loop.














    






        








