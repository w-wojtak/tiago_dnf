#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import json
from std_msgs.msg import Float32MultiArray, String
from time import time
import re

class VisionToDNFAggregator:
    def __init__(self):
        rospy.init_node('vision_to_dnf_aggregator_node', anonymous=True)

        self.x_lim, self.t_lim = 80, 15
        self.dx, self.dt = 0.2, 0.1
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.t = np.arange(0, self.t_lim + self.dt, self.dt)

        self.pub_inputs = rospy.Publisher('/dnf_inputs', Float32MultiArray, queue_size=10)
        self.sub_detections = rospy.Subscriber('/object_detections', String, self.detection_callback, queue_size=10)
        
        # This set is the single source of truth for active voice commands.
        self.active_human_voice_objects = set()
        
        rospy.Subscriber('/simulation/robot_feedback', String, self.robot_feedback_callback)
        self.sub_response_command = rospy.Subscriber('/response_command', String, self.response_command_callback)

        # Subscribing to the simple topic from the parser node
        rospy.Subscriber('/parsed_voice_command', String, self.voice_command_callback)

        # manual feedback to create inputs to u_f1 in case they don't come from the robot
        rospy.Subscriber('/manual_robot_feedback', String, self.manual_robot_feedback_callback)

        self.accumulated_robot_feedback = np.zeros_like(self.x)
        self.amplitude = 5.0
        self.width = 2.0
        self.duration = 1.0
        self.object_positions = {'base': -60, 'load': -20, 'bearing': 20, 'motor': 40}
        self.last_positions = {}
        self.pickup_detection_threshold = 0.1
        self.picked_up_objects = set()
        self.active_gaussians = []
        self.input_matrix1 = np.zeros((len(self.t), len(self.x)))
        self.input_matrix2 = np.zeros((len(self.t), len(self.x)))
        self.current_time_index = 0
        self.start_time = None
        self.main_publishing_timer = None

        rospy.loginfo("Vision→DNF aggregator node initialized.")
        rospy.Timer(rospy.Duration(0.5), self.wait_for_subscriber_and_start, oneshot=True)


    def manual_robot_feedback_callback(self, msg):
        """Receive manual robot feedback and apply exactly as automatic feedback."""
        object_name = msg.data
        if object_name in self.object_positions:
            center = self.object_positions[object_name]
            rospy.loginfo(f"Aggregator: Received MANUAL robot feedback for '{object_name}'. Applying Gaussian.")
            new_gaussian = self.gaussian(center=center, amplitude=5.0, width=2.0)
            self.accumulated_robot_feedback += new_gaussian


    def response_command_callback(self, msg):
        match = re.search(r"'(.+?)'", msg.data)
        if match:
            object_name = match.group(1)
            rospy.loginfo(f"************** Resetting (deactivating) human voice input for '{object_name}'.")
            if object_name in self.active_human_voice_objects:
                self.active_human_voice_objects.remove(object_name)


    def voice_command_callback(self, msg):
        """Receives a simple object name and activates its Gaussian input."""
        object_name = msg.data
        if object_name in self.object_positions:
            if object_name not in self.active_human_voice_objects:
                rospy.loginfo(f"Activating human voice input for '{object_name}'.")
                self.active_human_voice_objects.add(object_name)
            else:
                rospy.loginfo(f"Voice input for '{object_name}' is already active.")
        else:
            rospy.logwarn(f"Received unknown object name '{object_name}' on /parsed_voice_command")


    def robot_feedback_callback(self, msg):
        object_name = msg.data
        if object_name in self.object_positions:
            center = self.object_positions[object_name]
            rospy.loginfo(f"Aggregator: Heard robot feedback for '{object_name}'. Creating PERMANENT Gaussian.")
            new_gaussian = self.gaussian(center=center, amplitude=5.0, width=2.0)
            self.accumulated_robot_feedback += new_gaussian

    def wait_for_subscriber_and_start(self, event):
        rospy.loginfo("Checking for subscribers...")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and self.pub_inputs.get_num_connections() == 0:
            rospy.loginfo("Waiting for a subscriber to connect to /dnf_inputs...")
            rate.sleep()
        if rospy.is_shutdown(): return
        rospy.loginfo("Subscriber connected! Starting the 1 Hz publishing timer.")
        self.main_publishing_timer = rospy.Timer(rospy.Duration(1.0), self.publish_slices)

    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        return amplitude * np.exp(-((self.x - center) ** 2) / (2 * (width ** 2)))

    def publish_slices(self, event):
        if self.start_time is None: self.start_time = time()
        if self.current_time_index < len(self.t):
            self.update_input_matrices()
            robot_input_for_this_step = self.accumulated_robot_feedback
            human_input_for_this_step = np.zeros_like(self.x)
            for object_name in self.active_human_voice_objects:
                if object_name in self.object_positions:
                    center = self.object_positions[object_name]
                    human_gaussian = self.gaussian(center=center, amplitude=5.0, width=2.0)
                    human_input_for_this_step += human_gaussian
            combined_input = [
                self.input_matrix1[self.current_time_index].tolist(),
                robot_input_for_this_step.tolist(),
                human_input_for_this_step.tolist()
            ]
            msg = Float32MultiArray()
            msg.data = [item for sublist in combined_input for item in sublist]
            self.pub_inputs.publish(msg)
            elapsed = time() - self.start_time
            sim_time = self.t[self.current_time_index]
            rospy.loginfo(
                f"Published [elapsed={elapsed:.1f}s, sim_t={sim_time:.1f}s] | "
                f"Max: vis={np.max(self.input_matrix1[self.current_time_index]):.2f}, "
                f"rob={np.max(robot_input_for_this_step):.2f}, "
                f"hum={np.max(human_input_for_this_step):.2f} | "
                f"Picked: {len(self.picked_up_objects)}/4"
            )
            self.current_time_index += 1
        else:
            rospy.loginfo("Completed publishing all time slices.")
            self.main_publishing_timer.shutdown()


    def calculate_movement(self, old_pos, new_pos):
        dx = new_pos['x'] - old_pos['x']; dy = new_pos['y'] - old_pos['y']; dz = new_pos['z'] - old_pos['z']
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def check_for_pickup(self, object_name, new_position):
        if object_name not in self.last_positions:
            self.last_positions[object_name] = new_position.copy(); return False
        distance = self.calculate_movement(self.last_positions[object_name], new_position)
        if distance > self.pickup_detection_threshold:
            self.last_positions[object_name] = new_position.copy(); return True
        return False

    def add_gaussian_input(self, object_name):
        if object_name not in self.object_positions: return
        current_time = self.t[self.current_time_index]
        t_start = current_time; t_stop = t_start + self.duration
        center = self.object_positions[object_name]
        gaussian_params = {'center': center, 'amplitude': self.amplitude, 'width': self.width, 't_start': t_start, 't_stop': t_stop}
        self.active_gaussians.append(gaussian_params.copy())
        rospy.loginfo(f"Added PICKUP gaussian for '{object_name}'")

    def update_input_matrices(self):
        current_time = self.t[self.current_time_index]
        self.input_matrix1[self.current_time_index] = np.zeros(len(self.x))
        self.input_matrix2[self.current_time_index] = np.zeros(len(self.x))
        active_gaussians_now = []
        for g in self.active_gaussians:
            if current_time <= g['t_stop']:
                active_gaussians_now.append(g)
                if g['t_start'] <= current_time:
                    profile = self.gaussian(center=g['center'], amplitude=g['amplitude'], width=g['width'])
                    self.input_matrix1[self.current_time_index] += profile
                    self.input_matrix2[self.current_time_index] += profile
        self.active_gaussians = active_gaussians_now

    def detection_callback(self, msg):
        try:
            detections = json.loads(msg.data).get('detections', [])
            for detection in detections:
                object_name = detection.get('object', 'Unknown')
                if object_name not in self.object_positions: continue
                if self.check_for_pickup(object_name, detection.get('position', {})) and object_name not in self.picked_up_objects:
                    rospy.loginfo(f"PICKUP DETECTED: '{object_name}'")
                    self.add_gaussian_input(object_name)
                    self.picked_up_objects.add(object_name)
        except Exception as e:
            rospy.logerr(f"Error processing detection message: {e}")

def main():
    try:
        VisionToDNFAggregator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()