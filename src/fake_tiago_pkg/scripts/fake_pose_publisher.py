#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('fake_pose_publisher', anonymous=False)
    pub = rospy.Publisher('/dxl_input/pos_right', PoseStamped, queue_size=10)
    rate = rospy.Rate(1.0)  # 1 Hz

    seq = 0
    rospy.loginfo("fake_pose_publisher started -> publishing /dxl_input/pos_right")

    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = random.uniform(-0.5, 0.5)
        msg.pose.position.y = random.uniform(-0.5, 0.5)
        msg.pose.position.z = random.uniform(0.2, 1.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        pub.publish(msg)
        rospy.loginfo("[pose #%d] x=%.3f y=%.3f z=%.3f", seq,
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
