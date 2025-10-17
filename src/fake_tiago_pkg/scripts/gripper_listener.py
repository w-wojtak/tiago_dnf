#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

def callback(msg):
    rospy.loginfo("Received gripper -> x: %.3f, y: %.3f, z: %.3f",
                  msg.point.x, msg.point.y, msg.point.z)

def main():
    rospy.init_node('gripper_listener', anonymous=True)
    rospy.Subscriber('/dxl_input/gripper_right', PointStamped, callback)
    rospy.loginfo("gripper_listener started, listening to /dxl_input/gripper_right")
    rospy.spin()

if __name__ == '__main__':
    main()
