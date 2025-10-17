#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped

class TestListener(object):
    def __init__(self):
        rospy.init_node('test_listener', anonymous=False)
        self.pose_sub = rospy.Subscriber('/dxl_input/pos_right', PoseStamped, self.pose_cb, queue_size=10)
        self.grip_sub = rospy.Subscriber('/dxl_input/gripper_right', PointStamped, self.grip_cb, queue_size=10)
        self.pose_count = 0
        self.grip_count = 0
        rospy.loginfo("test_listener started: subscribing to /dxl_input/pos_right and /dxl_input/gripper_right")

    def pose_cb(self, msg: PoseStamped):
        self.pose_count += 1
        hdr = msg.header
        pos = msg.pose.position
        # Print concise, human-readable info
        rospy.loginfo("[pose #%d] seq=%s time=%s frame_id=%s -> x=%.3f y=%.3f z=%.3f",
                      self.pose_count, hdr.seq, hdr.stamp.to_sec(), hdr.frame_id, pos.x, pos.y, pos.z)

    def grip_cb(self, msg: PointStamped):
        self.grip_count += 1
        hdr = msg.header
        p = msg.point
        rospy.loginfo("[grip #%d] seq=%s time=%s frame_id=%s -> x=%.3f y=%.3f z=%.3f",
                      self.grip_count, hdr.seq, hdr.stamp.to_sec(), hdr.frame_id, p.x, p.y, p.z)

if __name__ == '__main__':
    try:
        node = TestListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
