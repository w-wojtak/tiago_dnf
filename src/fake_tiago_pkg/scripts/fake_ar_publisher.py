#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that simulates a QR code detector (like ar_track_alvar).
- Publishes the current state of all QR codes as ar_track_alvar_msgs/AlvarMarkers.
- Publishes to /ar_pose_marker.
- This node is REACTIVE. It listens for messages telling it that an object has
  been picked up and updates its internal world state accordingly.
"""

import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, Point, Quaternion

class FakeARPublisher:
    def __init__(self):
        rospy.init_node('fake_ar_publisher_node', anonymous=True)

        # --- CONFIGURATION ---
        self.publish_topic = '/ar_pose_marker'
        self.publish_rate_hz = 5.0
        self.camera_frame_id = 'camera_rgb_optical_frame'

        # Mapping of object names to their QR code IDs
        self.object_to_qr_id = {
            'base': 10,
            'load': 25,
            'bearing': 42,
            'motor': 55
        }

        # Initial poses of objects "on the table" relative to the camera
        self.initial_poses = {
            'base':    Pose(position=Point(x=-0.3, y=0.1, z=0.8), orientation=Quaternion(w=1.0)),
            'load':    Pose(position=Point(x=-0.1, y=0.1, z=0.8), orientation=Quaternion(w=1.0)),
            'bearing': Pose(position=Point(x=0.1, y=0.1, z=0.8), orientation=Quaternion(w=1.0)),
            'motor':   Pose(position=Point(x=0.3, y=0.1, z=0.8), orientation=Quaternion(w=1.0))
        }
        
        # Pose of an object after it has been "picked up" (far away and out of sight)
        self.picked_up_pose = Pose(position=Point(x=0.0, y=1.5, z=1.5), orientation=Quaternion(w=1.0))

        # --- STATE ---
        # This dictionary holds the current pose for each object. It's modified by callbacks.
        self.current_object_poses = self.initial_poses.copy()

        # --- ROS SETUP ---
        self.pub = rospy.Publisher(self.publish_topic, AlvarMarkers, queue_size=10)
        
        # --- SUBSCRIBERS ---
        # This node listens to two topics. One for simulated human actions (during learning)
        # and one for simulated robot actions (during recall).
        rospy.Subscriber('/simulation/human_pickup', String, self.pickup_callback)
        rospy.Subscriber('/simulation/robot_pickup', String, self.pickup_callback)

        # Start the main publishing loop to continuously publish the world state
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.publish_markers)
        
        rospy.loginfo("REACTIVE Fake AR Marker Publisher started.")
        rospy.loginfo(f"Publishing world state to '{self.publish_topic}' at {self.publish_rate_hz} Hz.")
        rospy.loginfo("Awaiting pickup event messages to update world state...")

    def pickup_callback(self, msg):
        """
        This function is called when ANYONE (human or robot) picks up an object.
        It updates the internal state of the simulation.
        """
        obj_name = msg.data
        rospy.loginfo(f"SIMULATOR: Heard that '{obj_name}' was picked up. Updating world state.")
        if obj_name in self.current_object_poses:
            self.current_object_poses[obj_name] = self.picked_up_pose
            rospy.loginfo(f"SIMULATOR: '{obj_name}' (ID {self.object_to_qr_id[obj_name]}) has been moved.")
        else:
            rospy.logwarn(f"SIMULATOR: Received pickup command for unknown object '{obj_name}'.")

    def publish_markers(self, event):
        """Constructs and publishes the AlvarMarkers message reflecting the current world state."""
        markers_msg = AlvarMarkers()
        markers_msg.header.stamp = rospy.Time.now()
        markers_msg.header.frame_id = self.camera_frame_id

        for obj_name, current_pose in self.current_object_poses.items():
            marker = AlvarMarker()
            marker.header.stamp = markers_msg.header.stamp
            marker.header.frame_id = markers_msg.header.frame_id
            marker.id = self.object_to_qr_id[obj_name]
            
            marker.pose.header = marker.header
            marker.pose.pose = current_pose
            
            # Confidence must be an integer
            marker.confidence = 0
            
            markers_msg.markers.append(marker)
        
        self.pub.publish(markers_msg)
        rospy.logdebug(f"Published state of {len(markers_msg.markers)} markers.")

def main():
    try:
        FakeARPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()