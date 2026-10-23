#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from tqdm import tqdm
import os
import signal

class WaypointPublisher:
    def __init__(self, waypoint_file):
        self.waypoints, self.waypoint_types = self.read_waypoints(waypoint_file)
        self.current_waypoint_index = 0
        self.current_pose = None
        self.hold_start_time = None
        self.hold_duration = 4.0  # Hold for 1 second on animal waypoints
        self.is_holding = False
        
        rospy.init_node('waypoint_publisher', anonymous=True)
        
        self.pub = rospy.Publisher('/command/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/ground_truth/state', Odometry, self.odometry_callback)
        
        self.rate = rospy.Rate(10)  # Publica a 10 Hz
        self.progress_bar = tqdm(total=len(self.waypoints), desc='Waypoints Progress')

    def read_waypoints(self, file_path):
        data = np.loadtxt(file_path, delimiter=',', skiprows=1)
        print(f"Loaded waypoint data shape: {data.shape}")
        
        if data.shape[1] >= 4:
            # New format: x,y,z,type
            waypoints = data[:, :3]  # x, y, z
            waypoint_types = data[:, 3]  # type (0=intermediate, 1=animal)
            print(f"Found {np.sum(waypoint_types == 1)} animal waypoints and {np.sum(waypoint_types == 0)} intermediate waypoints")
        else:
            # Old format: x,y,z (assume all are animal waypoints)
            waypoints = data
            waypoint_types = np.ones(len(waypoints))
            print(f"Old format detected - treating all {len(waypoints)} waypoints as animal waypoints")
        
        return waypoints, waypoint_types

    def odometry_callback(self, data):
        self.current_pose = data.pose.pose

    def distance_to_waypoint(self, waypoint):
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.position.x - waypoint[0]
        dy = self.current_pose.position.y - waypoint[1]
        dz = self.current_pose.position.z - waypoint[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_waypoints(self):
        while not rospy.is_shutdown():
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                waypoint_type = self.waypoint_types[self.current_waypoint_index]
                
                distance = self.distance_to_waypoint(waypoint)
                
                # Check if we've reached the waypoint
                if distance < 0.2:  # 40 cm tolerance
                    if not self.is_holding and waypoint_type == 1:  # Animal waypoint
                        # Start holding at animal waypoint
                        self.is_holding = True
                        self.hold_start_time = rospy.Time.now()
                        rospy.loginfo(f"Reached ANIMAL waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}) - Holding for {self.hold_duration}s")
                    elif not self.is_holding and waypoint_type == 0:  # Intermediate waypoint
                        # No holding for intermediate waypoints
                        rospy.loginfo(f"Reached intermediate waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}) - Continuing")
                        self.current_waypoint_index += 1
                        self.progress_bar.update(1)
                        
                # Check if holding period is complete
                if self.is_holding:
                    elapsed_time = (rospy.Time.now() - self.hold_start_time).to_sec()
                    if elapsed_time >= self.hold_duration:
                        rospy.loginfo(f"Hold complete - Moving to next waypoint")
                        self.is_holding = False
                        self.hold_start_time = None
                        self.current_waypoint_index += 1
                        self.progress_bar.update(1)
                
                # Check if all waypoints completed
                if self.current_waypoint_index >= len(self.waypoints):
                    rospy.loginfo("All waypoints reached.")
                    self.progress_bar.close()
                    self.shutdown_sequence()
                    break
                
                # Update waypoint if we moved to next one
                if self.current_waypoint_index < len(self.waypoints):
                    waypoint = self.waypoints[self.current_waypoint_index]

                # Publish current waypoint command
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'world'
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0  # No rotation

                self.pub.publish(pose)
            self.rate.sleep()

    def shutdown_sequence(self):

        rospy.loginfo("Shutting down Gazebo...")
        os.system('pkill gzserver')
        os.system('pkill gzclient')
        
        rospy.loginfo("Shutting down ROS...")
        os.system('pkill roscore')
        os.system('pkill rosmaster')
        
        rospy.loginfo("Shutting down the program...")
        os.kill(os.getpid(), signal.SIGINT)

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoint_file = rospy.get_param('~waypoint_file', 'zigzag_path.txt')
    
    try:
        waypoint_publisher = WaypointPublisher(waypoint_file)
        waypoint_publisher.publish_waypoints()
    except rospy.ROSInterruptException:
        pass
