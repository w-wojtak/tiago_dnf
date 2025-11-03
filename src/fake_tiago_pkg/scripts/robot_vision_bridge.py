#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node to bridge individual QR tag detections to the DNF system.

Now subscribes to four separate pose topics (one per tag), and publishes a JSON
message to /object_detections when a tag *first becomes visible*.
"""

import rospy
import json
from std_msgs.msg import String
from datetime import datetime
from geometry_msgs.msg import PoseStamped


class RobotVisionBridge:
    def __init__(self):
        rospy.init_node('robot_vision_bridge_node', anonymous=True)

        # 4 topics, 4 subscribers, 1 shared callback
        # each tag publishes only once when it first appears

        # outputs for vision_to_dnf_aggregator
        # removed position from detections-not needed
        self.output_topic = '/object_detections'

        # Mapping from tag ID to object names
        self.qr_to_object_map = {
            36: 'base',
            7: 'load',
            20: 'bearing',
            23: 'motor'
        }

        # example message
        # rostopic hz /orbbec_head/pose_tag_20

        # Keep track of which tags were already published (appear only once)
        self.already_published_ids = set()

        # --- ROS SETUP ---
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=10)

        # Subscribe to each tag topic
        for tag_id in self.qr_to_object_map.keys():
            topic_name = f"/orbbec_head/pose_tag_{tag_id}"
            # this automatically creates 4 subscribers
            rospy.Subscriber(topic_name, PoseStamped, self.pose_callback, callback_args=tag_id)
            rospy.loginfo(f"Subscribed to: {topic_name}")

        rospy.loginfo(f"Publishing detections to: {self.output_topic}")

    # eg when /orbbec_head/pose_tag_20 publishes something ROS automatically calls
    # pose_callback(msg, 20)

    # --------------------------------------------------------------------------
    def pose_callback(self, msg, tag_id):
        """Called when a tag-specific topic publishes (i.e., tag is visible)."""
        # Only process the first time this tag is seen
        if tag_id in self.already_published_ids:
            return

        self.already_published_ids.add(tag_id)
        object_name = self.qr_to_object_map[tag_id]

        # Construct output JSON
        current_time = rospy.Time.now()
        output_msg = {
            "timestamp": current_time.to_sec(),
            "frame_time": datetime.now().isoformat() + 'Z',
            "detections": [{"object": object_name}]
        }

        msg_str = String()
        msg_str.data = json.dumps(output_msg)
        self.pub.publish(msg_str)

        rospy.loginfo(f"Marker for '{object_name}' (tag {tag_id}) just became visible!")

    # --------------------------------------------------------------------------
def main():
    try:
        RobotVisionBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
