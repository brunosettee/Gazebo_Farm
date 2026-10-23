#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==============================================================================
# IMPORTS
# ==============================================================================
import os
import rospy
import cv2
import tf
import random
from math import *
import numpy as np
# Import specific ROS message types required for subscribing to topics
from sensor_msgs.msg import Range, Image, NavSatFix, CameraInfo
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge # Used to convert between ROS Image messages and OpenCV images
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion # Used to convert orientation from quaternion to Euler angles

# ==============================================================================
# GLOBAL CONFIGURATION & VARIABLES
# ==============================================================================

# --- Timings ---
last_save_time = 0  # Timestamp of the last data save
save_interval = 10.0 # Time interval in seconds for saving data.

# --- Noise Parameters ---
# These standard deviations are used to add Gaussian noise to sensor readings,
# simulating real-world sensor inaccuracies. This is useful for creating
# more realistic datasets for testing and training robotics algorithms.
noise_std_lat = 0.00005     # Standard deviation for latitude noise
noise_std_lon = 0.00005     # Standard deviation for longitude noise
noise_std_alt = 0.05        # Standard deviation for altitude noise
noise_std_linear = 0.05     # Standard deviation for linear velocity noise
noise_std_angular = 0.05    # Standard deviation for angular velocity noise
noise_std_sonar = 0.05      # Standard deviation for sonar height noise

# --- Global Data Placeholders ---
# These variables will store the most recent message received from each topic.
# They are initialized to None and updated by their respective callback functions.
sonar_data = None
image_data = None
velocity_data = None
wind_data = None
gps_data = None
odometry_data = None
initial_pose_data = None
camera_params_data = None

# ==============================================================================
# IMAGE SAVING FUNCTION
# ==============================================================================

def save_image(simulation_time, image_data, caminho_output):
    """
    Converts a ROS Image message to an OpenCV image and saves it to a file.
    The filename is based on the simulation time, padded with leading zeros for sorting.
    """
    if image_data is None:
        rospy.logwarn("Image data is missing. Cannot save image.")
        return

    # Create the target directory if it doesn't exist
    folder_path = os.path.join(caminho_output, "images-file")
    os.makedirs(folder_path, exist_ok=True)

    # Format the simulation time to create a consistent filename
    # e.g., 12.345 -> "000000012.345"
    p = "0" * 12
    decimal_formatado = f"{simulation_time:.3f}"
    tempo_string = str(decimal_formatado)
    tempo_formatado = p[:(len(p) - len(tempo_string))] + tempo_string
    filename = f"image_{tempo_formatado}.png"
    full_path = os.path.join(folder_path, filename)

    try:
        bridge = CvBridge()
        # Convert the ROS image message to an OpenCV image format (bgr8 is standard for color)
        cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding="bgr8")
        # Save the OpenCV image to the specified path
        cv2.imwrite(full_path, cv_image)
        #rospy.loginfo(f"[SUCCESS] Image saved to: {full_path}")
    except Exception as e:
        rospy.logwarn(f"[ERROR] Failed to save image: {e}")

# ==============================================================================
# ROS CALLBACK FUNCTIONS
# ==============================================================================
# These functions are executed whenever a new message arrives on a subscribed topic.
# Their primary role is to update the global variables with the latest sensor data.

def gps_callback(data):
    """Stores the latest GPS data from the /fix topic."""
    global gps_data
    gps_data = data

def image_callback(data):
    """Stores the latest Image data from the /front_cam/camera/image topic."""
    global image_data
    #rospy.loginfo("IMAGEM RECEBIDA no callback!")
    image_data = data

def sonar_height_callback(data):
    """Stores the latest Sonar data from the /sonar_height topic."""
    global sonar_data
    sonar_data = data

def velocity_callback(data):
    """Stores the latest Velocity data from the /fix_velocity topic."""
    global velocity_data
    velocity_data = data

def wind_callback(data):
    """Stores the latest Wind data from the /wind topic."""
    global wind_data
    wind_data = data

def odometry_callback(data):
    """Stores the latest Odometry data from the /ground_truth/state topic."""
    global odometry_data
    odometry_data = data

def initial_pose_callback(data):
    """Stores the Initial Pose data from the /initialpose topic."""
    global initial_pose_data
    initial_pose_data = data

def camera_callback(data):
    """Stores the camera parameters from the /front_cam/camera/camera_info topic."""
    global camera_params_data
    camera_params_data = data

# ==============================================================================
# MAIN CONTROL AND SAVING LOGIC
# ==============================================================================

def clock_callback(msg, caminho_output):
    """
    Acts as the main loop trigger, called with each simulation clock update.
    It checks if enough time has passed to save a new batch of data.
    """
    global last_save_time, sonar_data, velocity_data, wind_data, image_data, gps_data, odometry_data, initial_pose_data, camera_params_data
    simulation_time = rospy.get_time()

    # Check if the save interval has elapsed since the last save
    if simulation_time - last_save_time >= save_interval:
        last_save_time = simulation_time
        # Call functions to save all collected data
        save_topic_info(simulation_time, sonar_data, velocity_data, wind_data, gps_data, odometry_data, initial_pose_data, caminho_output)
        save_image(simulation_time, image_data, caminho_output)
        save_camera_info(simulation_time, camera_params_data, caminho_output)

def check_and_delete_empty_file(file_path):
    """
    Checks if a file was created but is empty. If so, it deletes the file.
    This prevents empty log files if no data was received for a given interval.
    """
    if os.path.exists(file_path) and os.path.getsize(file_path) == 0:
        os.remove(file_path)
        rospy.loginfo(f"Empty file removed: {file_path}")

def save_topic_info(simulation_time, sonar_data, velocity_data, wind_data, gps_data, odometry_data, initial_pose_data, caminho_output):
    """
    Aggregates all sensor data into a single text file for a given timestamp.
    It also adds simulated Gaussian noise to certain measurements.
    """
    folder_path = os.path.join(caminho_output, "info-file")
    os.makedirs(folder_path, exist_ok=True) # Create directory if it doesn't exist
    
    # Format the filename based on simulation time, same as the image filename
    p = "0" * 12
    decimal_formatado = f"{simulation_time:.3f}"
    tempo_string = str(decimal_formatado)
    tempo_formatado = p[:(len(p) - len(tempo_string))] + tempo_string
    filename = f"simulation_{tempo_formatado}.txt"
    full_path = os.path.join(folder_path, filename)
    
    # Open the file and write the available data
    with open(full_path, "w") as file_handle:
        if sonar_data:
            file_handle.write(f"Sonar Height: {sonar_data.range}\n")
            # Add Gaussian noise to the sonar reading
            file_handle.write(f"Sonar Height with noise: {sonar_data.range + random.gauss(0, noise_std_sonar)}\n")

        if velocity_data:
            file_handle.write(f"\nVelocity - Linear: x={velocity_data.vector.x}, y={velocity_data.vector.y}, z={velocity_data.vector.z}\n")
            # Add Gaussian noise to each component of the linear velocity
            noisy_linear_x = velocity_data.vector.x + random.gauss(0, noise_std_linear)
            noisy_linear_y = velocity_data.vector.y + random.gauss(0, noise_std_linear)
            noisy_linear_z = velocity_data.vector.z + random.gauss(0, noise_std_linear)
            file_handle.write(f"Noisy Velocity - Linear: x={noisy_linear_x}, y={noisy_linear_y}, z={noisy_linear_z}\n")
            
        if wind_data:
            file_handle.write(f"\nWind - Linear: x={wind_data.x}, y={wind_data.y}, z={wind_data.z}\n")

        if gps_data:
            file_handle.write(f"\nGPS without noise:\n")
            file_handle.write(f"Altitude: {gps_data.altitude} Latitude: {gps_data.latitude}, Longitude: {gps_data.longitude}\n")
            # Add Gaussian noise to GPS coordinates
            file_handle.write(f"\nGPS with noise:\n")
            noisy_latitude = gps_data.latitude + random.gauss(0, noise_std_lat)
            noisy_longitude = gps_data.longitude + random.gauss(0, noise_std_lon)
            noisy_altitude = gps_data.altitude + random.gauss(0, noise_std_alt)
            file_handle.write(f"Altitude: {noisy_altitude} Latitude: {noisy_latitude}, Longitude: {noisy_longitude}\n")

        if odometry_data:
            # Extract quaternion from odometry data
            orientation_quaternion = odometry_data.pose.pose.orientation
            orientation_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
            # Convert quaternion to Euler angles (roll, pitch, yaw) for easier interpretation
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            
            file_handle.write(f"\nOrientation quaternion: \n")
            file_handle.write(f"x: {orientation_quaternion.x}\n")
            file_handle.write(f"y: {orientation_quaternion.y}\n")
            file_handle.write(f"z: {orientation_quaternion.z}\n")
            file_handle.write(f"w: {orientation_quaternion.w}\n")
            
            file_handle.write(f"Roll: {roll}\n")
            file_handle.write(f"Pitch: {pitch}\n")
            file_handle.write(f"Yaw: {yaw}\n")

            file_handle.write(f"\nOdometry Pose:\n")
            file_handle.write(f"X: {odometry_data.pose.pose.position.x}\n")
            file_handle.write(f"Y: {odometry_data.pose.pose.position.y}\n")
            file_handle.write(f"Z: {odometry_data.pose.pose.position.z}\n")

        if initial_pose_data:
            file_handle.write(f"\nInitial Pose:\n")
            file_handle.write(f"Position:\n")
            file_handle.write(f"x: {initial_pose_data.pose.pose.position.x}\n")
            file_handle.write(f"y: {initial_pose_data.pose.pose.position.y}\n")
            file_handle.write(f"z: {initial_pose_data.pose.pose.position.z}\n")

    # After writing, check if the file is empty and delete it if so
    check_and_delete_empty_file(full_path)

def save_camera_info(simulation_time, camera_params_data, caminho_output):
    """
    Saves the camera's intrinsic parameters (calibration data) to a file.
    This data is usually static, so the file will be overwritten at each interval,
    which is acceptable as the content does not change.
    """
    folder_path = os.path.join(caminho_output, "camera_info_file")
    os.makedirs(folder_path, exist_ok=True) # Create directory if it doesn't exist
    filename = "camera_params.txt"
    full_path = os.path.join(folder_path, filename)
    
    with open(full_path, "w") as file_handle:
        if camera_params_data:
            file_handle.write(f"height: {camera_params_data.height}\n")
            file_handle.write(f"width: {camera_params_data.width}\n")
            file_handle.write(f"distortion_model: {camera_params_data.distortion_model}\n")
            file_handle.write(f"D (Distortion Coefficients): {camera_params_data.D}\n")
            file_handle.write(f"K (Intrinsic Matrix): {camera_params_data.K}\n")
            file_handle.write(f"R (Rectification Matrix): {camera_params_data.R}\n")
            file_handle.write(f"P (Projection Matrix): {camera_params_data.P}\n")
            file_handle.write(f"binning_x: {camera_params_data.binning_x}\n")
            file_handle.write(f"binning_y: {camera_params_data.binning_y}\n")
            file_handle.write(f"roi (Region of Interest):\n")
            file_handle.write(f"  x_offset: {camera_params_data.roi.x_offset}\n")
            file_handle.write(f"  y_offset: {camera_params_data.roi.y_offset}\n")
            file_handle.write(f"  height: {camera_params_data.roi.height}\n")
            file_handle.write(f"  width: {camera_params_data.roi.width}\n")
            file_handle.write(f"  do_rectify: {camera_params_data.roi.do_rectify}\n")


# ==============================================================================
# NODE INITIALIZATION AND SUBSCRIBERS
# ==============================================================================

def hector_listener():
    """
    Initializes the ROS node and sets up all the subscribers to listen to sensor topics.
    """
    # Initialize the node with the name 'hector_listener'. anonymous=True ensures
    # the node has a unique name by adding a random number to the end.
    rospy.init_node('hector_listener', anonymous=True)

    # Get the output path from a private ROS parameter. This allows the user
    # to specify the output directory when launching the node.
    caminho_output = rospy.get_param('~caminho_output', '')

    # --- Setup Subscribers ---
    # Each subscriber listens to a specific topic, expects a certain message type,
    # and calls a specific function (callback) when a message is received.
    
    # The clock callback is special; it's used as the main timer for saving data.
    # We pass 'caminho_output' as an argument to the callback.
    rospy.Subscriber("/clock", Float64, clock_callback, callback_args=caminho_output)
    
    # Subscribers for various sensor data
    rospy.Subscriber("/sonar_height", Range, sonar_height_callback)
    rospy.Subscriber("/fix_velocity", Vector3Stamped, velocity_callback)
    rospy.Subscriber("/wind", Vector3, wind_callback)
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/ground_truth/state", Odometry, odometry_callback)
    rospy.Subscriber("/front_cam/camera/image", Image, image_callback)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initial_pose_callback)
    #rospy.Subscriber("/front_cam/camera/camera_info", CameraInfo, camera_callback)
    rospy.Subscriber("/downward_cam/camera/image", Image, image_callback)
    rospy.Subscriber("/downward_cam/camera/camera_info", CameraInfo, camera_callback)
    
    # rospy.spin() keeps the node from exiting and allows it to continue
    # listening for and processing messages.
    rospy.spin()

# Standard Python entry point.
if __name__ == '__main__':
    try:
        hector_listener()
    except rospy.ROSInterruptException:
        # This exception is raised when the node is shut down
        pass