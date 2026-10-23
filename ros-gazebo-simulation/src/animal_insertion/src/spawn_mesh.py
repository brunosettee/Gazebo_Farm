#!/usr/bin/env python3

import rospy
import os
import shutil
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from PIL import Image
import cv2
import numpy as np
import collada

def spawn_mesh(coordinates_path, model_path):
    rospy.init_node('spawn_mesh', anonymous=True)

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open(coordinates_path, 'r') as file:
        for line in file:
            coords = line.strip().split()
            if len(coords) != 8: 
                continue

            x, y, z = float(coords[0].strip(',')), float(coords[1].strip(',')), float(coords[2].strip(','))
            wq , xq, yq, zq = float(coords[3].strip(',')), float(coords[4].strip(',')), float(coords[5].strip(',')), float(coords[6].strip(','))
            model = coords[7].strip(',')

            mesh_id = f"Mesh_{x}_{y}_{z}"
            new_model_path = os.path.join(model_path, model)

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = xq
            pose.orientation.y = yq
            pose.orientation.z = zq
            pose.orientation.w = wq

            model_sdf = f"""
            <sdf version="1.6">
                <model name="{mesh_id}">
                    <pose>0 0 0 0 0 0</pose>
                    <link name="link_{model}_{x}_{y}_{z}">
                        <visual name="visual_{x}_{y}_{z}">
                            <geometry>
                                <mesh>
                                    <uri>file://{new_model_path}</uri>
                                    <scale>1 1 1</scale>
                                </mesh>
                            </geometry>
                        </visual>
                    </link>
                    <static>true</static>
                </model>
            </sdf>
            """
            try:
                spawn_model(mesh_id, model_sdf, '', pose, 'world')
                rospy.loginfo(f"Spawned mesh at ({x}, {y}, {z})")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        coordinates_path = '/catkin_ws/src/maps_file/mesh_files/map_0/coordinates.txt' 
        model_path = '/catkin_ws/src/maps_file/mesh_files/map_0/Models'
        spawn_mesh(coordinates_path, model_path)
    except rospy.ROSInterruptException:
        pass