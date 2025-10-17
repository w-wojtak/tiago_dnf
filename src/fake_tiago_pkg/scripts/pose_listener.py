#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    rospy.loginfo("Received pose -> x: %.3f, y: %.3f, z: %.3f",
                  msg.pose.position.x,
                  msg.pose.position.y,
                  msg.pose.position.z)

def main():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/dxl_input/pos_right', PoseStamped, callback)
    rospy.loginfo("pose_listener started, listening to /dxl_input/pos_right")
    rospy.spin()  # keeps node alive

if __name__ == '__main__':
    main()

