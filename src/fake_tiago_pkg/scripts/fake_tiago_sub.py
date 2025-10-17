#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")

def main():
    rospy.init_node('fake_tiago_sub')
    rospy.Subscriber('/tiago/fake_cmd', String, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
