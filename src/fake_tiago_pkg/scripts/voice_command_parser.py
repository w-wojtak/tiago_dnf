#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that parses raw voice messages into simple object names.
Subscribes to /voice_message (std_msgs/String).
Publishes the corresponding object name on /parsed_voice_command (std_msgs/String).
"""

import rospy
from std_msgs.msg import String

class VoiceCommandParser:
    def __init__(self):
        rospy.init_node('voice_command_parser_node', anonymous=True)

        # --- CHANGED: This map now translates from a command to a simple object name ---
        self.command_to_object = {
            "give_motor": "motor",
            "give_load": "load",
            "give_bearing": "bearing",
            "give_base": "base"
        }

        # --- CHANGED: Publisher now sends a simple String with the object name ---
        self.pub = rospy.Publisher('/parsed_voice_command', String, queue_size=10)

        # Subscriber to incoming raw voice commands
        rospy.Subscriber('/voice_message', String, self.voice_callback)

        rospy.loginfo("Voice Command Parser initialized. Listening on /voice_message...")

    def voice_callback(self, msg):
        cmd = msg.data.strip().lower()

        # --- CHANGED: Logic is much simpler ---
        # 1. Find the object name from the command
        # 2. Publish it.
        if cmd in self.command_to_object:
            object_name = self.command_to_object[cmd]
            
            # Publish the parsed object name
            self.pub.publish(object_name)
            
            rospy.loginfo(f"Parsed command '{cmd}' -> publishing object name '{object_name}'")
        
        else:
            # This makes the node more flexible if you just say "motor"
            found_object = None
            for key, val in self.command_to_object.items():
                if val in cmd:
                    found_object = val
                    break
            
            if found_object:
                 self.pub.publish(found_object)
                 rospy.loginfo(f"Parsed command '{cmd}' -> publishing object name '{found_object}'")
            else:
                rospy.logwarn(f"Unknown voice command '{cmd}' - ignoring")

if __name__ == "__main__":
    try:
        node = VoiceCommandParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass