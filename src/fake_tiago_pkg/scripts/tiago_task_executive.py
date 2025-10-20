#!/usr/bin/env python3
import rospy
import threading
import math
from std_msgs.msg import String, Float32MultiArray, Header # <-- ADD Header
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion

class TiagoTaskExecutive:
    def __init__(self):
        rospy.init_node('tiago_task_executive', anonymous=True)

        # --- Mappings ---
        self.DNF_POS_TO_OBJECT = {-60.0: 'base', -20.0: 'load', 20.0: 'bearing', 40.0: 'motor'}
        default_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.OBJECT_POSES = {
            'base':    Pose(position=Point(x=0.5, y= 0.3, z=0.1), orientation=default_orientation),
            'load':    Pose(position=Point(x=0.5, y= 0.1, z=0.1), orientation=default_orientation),
            'bearing': Pose(position=Point(x=0.5, y=-0.1, z=0.1), orientation=default_orientation),
            'motor':   Pose(position=Point(x=0.5, y=-0.3, z=0.1), orientation=default_orientation),
        }
        self.PRE_GRASP_HEIGHT = 0.25
        self.HOME_POSE = Pose(position=Point(x=0.3, y=0.0, z=0.4), orientation=default_orientation)

        # Publishers, Subscribers, etc.
        self.pose_cmd_pub = rospy.Publisher('/dxl_input/pos_right', PoseStamped, queue_size=10)
        self.gripper_cmd_pub = rospy.Publisher('/dxl_input/gripper_right', PointStamped, queue_size=10)
        self.pickup_announcement_pub = rospy.Publisher('/simulation/robot_pickup', String, queue_size=10)
        self.robot_feedback_pub = rospy.Publisher('/simulation/robot_feedback', String, queue_size=10)
        rospy.Subscriber('threshold_crossings', Float32MultiArray, self.dnf_prediction_callback)
        self.current_robot_pose = None
        self.pose_lock = threading.Lock()
        rospy.Subscriber('/cartesian/right_arm/pose', PoseStamped, self.robot_pose_callback)

        rospy.loginfo("TIAGO Task Executive started. Awaiting DNF predictions and robot pose feedback.")

    def robot_pose_callback(self, msg):
        with self.pose_lock:
            self.current_robot_pose = msg.pose

    def dnf_prediction_callback(self, msg):
        if not msg.data: return
        dnf_pos = msg.data[0]
        object_name = self.DNF_POS_TO_OBJECT.get(dnf_pos)
        if object_name:
            if hasattr(self, '_active_thread') and self._active_thread.is_alive():
                rospy.logwarn(f"Executive: Ignoring new prediction for '{object_name}' as a sequence is already running.")
                return
            self._active_thread = threading.Thread(target=self.handle_pickup_sequence, args=(object_name,))
            self._active_thread.start()

    def handle_pickup_sequence(self, object_name):
        rospy.loginfo(f"====== STARTING TIAGO PICKUP SEQUENCE FOR '{object_name}' ======")
        target_obj_pose = self.OBJECT_POSES.get(object_name)
        if not target_obj_pose:
            rospy.logerr(f"No pose defined for object '{object_name}'!")
            return

        pre_grasp_pose = Pose(position=Point(x=target_obj_pose.position.x, y=target_obj_pose.position.y, z=self.PRE_GRASP_HEIGHT), orientation=target_obj_pose.orientation)
        grasp_pose = Pose(position=Point(x=target_obj_pose.position.x, y=target_obj_pose.position.y, z=target_obj_pose.position.z), orientation=target_obj_pose.orientation)
        post_grasp_pose = pre_grasp_pose

        try:
            self._set_gripper_and_wait("open")
            self._move_arm_and_wait(pre_grasp_pose)
            self._move_arm_and_wait(grasp_pose)
            self._set_gripper_and_wait("close")
            self._move_arm_and_wait(post_grasp_pose)
            self._move_arm_and_wait(self.HOME_POSE)
            rospy.loginfo(f"====== TIAGO PICKUP SEQUENCE FOR '{object_name}' COMPLETE ======")
            self.pickup_announcement_pub.publish(String(data=object_name))
            self.robot_feedback_pub.publish(String(data=object_name))
        except Exception as e:
            rospy.logerr(f"CRASH IN TIAGO PICKUP SEQUENCE: {e}")
    
    def _move_arm_and_wait(self, target_pose, timeout=10.0):
        rospy.loginfo(f"Executive: Commanding arm to z={target_pose.position.z:.2f}")
        
        # === START OF CORRECTION ===
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose = target_pose
        # === END OF CORRECTION ===

        self.pose_cmd_pub.publish(pose_msg)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            with self.pose_lock:
                if self.current_robot_pose is None:
                    rospy.logwarn_throttle(2.0, "Waiting for first robot pose feedback on /cartesian/right_arm/pose...")
                    rate.sleep()
                    continue
                dist = math.sqrt(sum([(getattr(self.current_robot_pose.position, axis) - getattr(target_pose.position, axis))**2 for axis in 'xyz']))
                if dist < 0.02:
                    rospy.loginfo("Executive: Arm has reached the target.")
                    return
            rate.sleep()
        raise Exception("Arm movement timed out")

    def _set_gripper_and_wait(self, state, delay=1.5):
        rospy.loginfo(f"Executive: Commanding gripper to '{state}'.")
        
        # === START OF CORRECTION ===
        gripper_msg = PointStamped()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.point = Point(x=1.0 if state == "open" else 0.0, y=0.0, z=0.0)
        # === END OF CORRECTION ===

        self.gripper_cmd_pub.publish(gripper_msg)
        rospy.sleep(delay)
        rospy.loginfo(f"Executive: Gripper action '{state}' complete.")

if __name__ == '__main__':
    try:
        TiagoTaskExecutive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass