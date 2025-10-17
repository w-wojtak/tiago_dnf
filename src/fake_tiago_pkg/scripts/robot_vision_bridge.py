#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node to bridge a robot's QR code detections to the DNF system.
- Subscribes to /ar_pose_marker (from ar_track_alvar or a simulator).
- Maps QR code IDs to object names.
- Publishes messages in the required JSON format to /object_detections.
"""

import rospy
import json
from std_msgs.msg import String
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers

class RobotVisionBridge:
    def __init__(self):
        rospy.init_node('robot_vision_bridge_node', anonymous=True)

        # --- CONFIGURATION ---
        self.output_topic = '/object_detections'
        self.qr_code_input_topic = '/ar_pose_marker'
        
        # Mapping from QR code ID to the object's name
        self.qr_to_object_map = {
            10: 'base',
            25: 'load',
            42: 'bearing',
            55: 'motor'
        }

        # Hardcoded table boundaries (can be adjusted)
        self.table_bounds = {
            'x_min': -0.82, 'x_max': 0.71,
            'y_min': 0.06, 'y_max': 0.52
        }
        
        # --- ROS SETUP ---
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(self.qr_code_input_topic, AlvarMarkers, self.qr_callback)
        
        rospy.loginfo("Robot Vision Bridge started.")
        rospy.loginfo(f"Listening for QR codes on: {self.qr_code_input_topic}")
        rospy.loginfo(f"Publishing to: {self.output_topic}")

    def qr_callback(self, msg):
        """Processes incoming QR code detections."""
        detections_list = []

        for marker in msg.markers:
            qr_id = marker.id
            if qr_id not in self.qr_to_object_map:
                continue
            
            object_name = self.qr_to_object_map[qr_id]
            pos = marker.pose.pose.position
            
            detection = {
                "object": object_name,
                "position": {
                    "x": round(pos.x, 3),
                    "y": round(pos.y, 3),
                    "z": round(pos.z, 3)
                }
            }
            detections_list.append(detection)
        
        if not detections_list:
            return

        current_time = rospy.Time.now()
        output_msg = {
            "timestamp": current_time.to_sec(),
            "frame_time": datetime.now().isoformat() + 'Z',
            "table_bounds": self.table_bounds,
            "detections": detections_list
        }

        msg_str = String()
        msg_str.data = json.dumps(output_msg)
        self.pub.publish(msg_str)

def main():
    try:
        RobotVisionBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()