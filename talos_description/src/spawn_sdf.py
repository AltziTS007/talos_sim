#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import os

# Initialize the ROS node
rospy.init_node('insert_object', log_level=rospy.INFO)

# Define the initial position of the object
initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 0.3

# Prompt the user to input the relative path to the model file
some_robo_name = input("Please enterEnter the name of the object (e.g., axis, S40_40G .. ): ")
relative_model_path = input("Please enter the relative path to the model file (e.g., talos_description/urdf/object_sdf/CONTAINER_RED/model.sdf): ")

# Convert the relative path to an absolute path
model_path = os.path.abspath(relative_model_path)

# Check if the file exists at the given path
if not os.path.isfile(model_path):
    print(f"Error: The file at path '{model_path}' does not exist.")
else:
    # Open and read the model file
    with open(model_path, 'r') as f:
        sdff = f.read()

    # Wait for the 'gazebo/spawn_sdf_model' service to be available
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    # Create a service proxy to spawn the model
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    # Call the service to spawn the model
    spawn_model_prox(some_robo_name, sdff, "RoboCupWork", initial_pose, "world")

