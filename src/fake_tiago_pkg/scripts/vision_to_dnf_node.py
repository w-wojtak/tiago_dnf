#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that translates object detections into DNF-compatible input matrices.
Publishes combined matrices on /dnf_inputs as Float32MultiArray messages.
Subscribes to /object_detections (std_msgs/String, JSON format).
Detects object movements (pickups) greater than 10cm and generates Gaussian inputs.
"""

import rospy
import numpy as np
import json
from std_msgs.msg import Float32MultiArray, String
from time import time

class VisionToDNF:
    def __init__(self):
        rospy.init_node('vision_to_dnf_node', anonymous=True)

        # Simulation parameters (must match DNF node)
        self.x_lim, self.t_lim = 80, 15
        self.dx, self.dt = 0.2, 0.1

        # Define spatial and temporal grids
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.t = np.arange(0, self.t_lim + self.dt, self.dt)

        # Publisher & subscriber
        self.pub_inputs = rospy.Publisher('/dnf_inputs', Float32MultiArray, queue_size=10)
        self.sub_detections = rospy.Subscriber('/object_detections', String, self.detection_callback, queue_size=10)

        # Accumulated inputs that persist forever once activated
        self.accumulated_human_voice = np.zeros_like(self.x)
        self.accumulated_robot_feedback = np.zeros_like(self.x)
        
        rospy.Subscriber('/simulation/robot_feedback', String, self.robot_feedback_callback)
        rospy.Subscriber('/voice_command', String, self.voice_command_callback)

        # Gaussian parameters
        self.amplitude = 5.0
        self.width = 2.0
        self.duration = 1.0  # 1 second duration (for pickup detection only)

        # Object → Gaussian center mapping (DNF spatial positions)
        self.object_positions = {'base': -60, 'load': -20, 'bearing': 20, 'motor': 40}
        
        # Store last known positions for movement detection
        self.last_positions = {}  # Will be populated as objects are detected
        
        # Single threshold for detecting a pickup (10cm = 0.1m)
        self.pickup_detection_threshold = 0.1  # meters
        
        # Track which objects have been picked up to ensure a one-time trigger
        self.picked_up_objects = set()

        # Lists to store active gaussians
        self.active_gaussians = []

        # Initialize input matrices
        self.input_matrix1 = np.zeros((len(self.t), len(self.x)))
        self.input_matrix2 = np.zeros((len(self.t), len(self.x)))
        self.input_matrix_3 = np.zeros((len(self.t), len(self.x)))

        # Initialize the current time index for publishing
        self.current_time_index = 0
        
        # Track actual elapsed time
        self.start_time = None

        # Main publishing timer (will be started later)
        self.main_publishing_timer = None

        rospy.loginfo("Vision→DNF node initialized. Will start publishing once a subscriber connects.")
        
        rospy.Timer(rospy.Duration(0.5), self.wait_for_subscriber_and_start, oneshot=True)

        rospy.loginfo(f"Pickup detection threshold set to: {self.pickup_detection_threshold}m (10cm)")


    def voice_command_callback(self, msg):
        """Receives a voice command and adds a permanent Gaussian input."""
        object_name = msg.data
        if object_name in self.object_positions:
            center = self.object_positions[object_name]
            rospy.loginfo(f"Aggregator: Heard voice command for '{object_name}'. Creating PERMANENT Gaussian for DNF.")
            new_gaussian = self.gaussian(center=center, amplitude=5.0, width=2.0)
            self.accumulated_human_voice += new_gaussian


    def robot_feedback_callback(self, msg):
        """Receives robot feedback and adds a permanent Gaussian input."""
        object_name = msg.data
        if object_name in self.object_positions:
            center = self.object_positions[object_name]
            rospy.loginfo(f"Aggregator: Heard robot feedback for '{object_name}'. Creating PERMANENT Gaussian.")
            new_gaussian = self.gaussian(center=center, amplitude=5.0, width=2.0)
            self.accumulated_robot_feedback += new_gaussian


    def wait_for_subscriber_and_start(self, event):
        """
        This function is called once after a short delay. It waits until a
        subscriber is connected to '/dnf_inputs' and then starts the main
        publishing timer.
        """
        rospy.loginfo("Checking for subscribers...")
        rate = rospy.Rate(2) # Check twice per second

        while not rospy.is_shutdown() and self.pub_inputs.get_num_connections() == 0:
            rospy.loginfo("Waiting for a subscriber to connect to /dnf_inputs...")
            rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo("Subscriber connected! Starting the 1 Hz publishing timer.")
        # Now that we have a subscriber, it's safe to start the main publishing loop.
        self.main_publishing_timer = rospy.Timer(rospy.Duration(1.0), self.publish_slices)


    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        """Generate Gaussian profile centered at 'center'"""
        return amplitude * np.exp(-((self.x - center) ** 2) / (2 * (width ** 2)))


    def calculate_movement(self, old_pos, new_pos):
        """Calculate the Euclidean distance between two 3D positions"""
        dx = new_pos['x'] - old_pos['x']
        dy = new_pos['y'] - old_pos['y']
        dz = new_pos['z'] - old_pos['z']
        return np.sqrt(dx**2 + dy**2 + dz**2)


    def check_for_pickup(self, object_name, new_position):
        """
        Check if an object has moved beyond the pickup threshold.
        Returns True if a pickup is detected, False otherwise.
        """
        # First time seeing this object? Store its position and do nothing.
        if object_name not in self.last_positions:
            self.last_positions[object_name] = new_position.copy()
            rospy.loginfo(f"First detection of '{object_name}' at ({new_position['x']:.3f}, {new_position['y']:.3f}, {new_position['z']:.3f})")
            return False
        
        # Calculate movement distance
        old_pos = self.last_positions[object_name]
        distance = self.calculate_movement(old_pos, new_position)
        
        # Check if movement exceeds the pickup threshold
        if distance > self.pickup_detection_threshold:
            self.last_positions[object_name] = new_position.copy()
            return True
        
        return False


    def add_gaussian_input(self, object_name):
        """Add a gaussian input for the specified object to both matrices"""
        if object_name not in self.object_positions:
            rospy.logwarn(f"Unknown object '{object_name}' - skipping Gaussian generation")
            return
            
        current_time = self.t[self.current_time_index]
        t_start = current_time
        t_stop = t_start + self.duration
        center = self.object_positions[object_name]
        
        gaussian_params = {
            'center': center,
            'amplitude': self.amplitude,
            'width': self.width,
            't_start': t_start,
            't_stop': t_stop,
        }
        
        # Add the same Gaussian to the list for both matrices
        self.active_gaussians.append(gaussian_params.copy())
        
        elapsed = time() - self.start_time if self.start_time else 0
        rospy.loginfo(f"Added PICKUP gaussian for '{object_name}' at x={center}, "
                     f"sim_t={t_start:.1f}s (elapsed={elapsed:.1f}s)")


    def update_input_matrices(self):
        """Update both input matrices based on their active gaussians"""
        current_time = self.t[self.current_time_index]
        
        # Reset the current time slice for both matrices
        self.input_matrix1[self.current_time_index] = np.zeros(len(self.x))
        self.input_matrix2[self.current_time_index] = np.zeros(len(self.x))
        
        # Filter out expired Gaussians and apply active ones
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
        """Callback for object detection messages"""
        try:
            detection_data = json.loads(msg.data)
            detections = detection_data.get('detections', [])
            
            for detection in detections:
                object_name = detection.get('object', 'Unknown')
                position = detection.get('position', {})
                
                # Check if this is a known object
                if object_name not in self.object_positions:
                    continue
                
                # Check if the object was picked up and hasn't been triggered before
                if self.check_for_pickup(object_name, position) and object_name not in self.picked_up_objects:
                    # Object was picked up!
                    elapsed = time() - self.start_time if self.start_time else 0
                    
                    rospy.loginfo(f"PICKUP DETECTED: '{object_name}' moved more than {self.pickup_detection_threshold}m (elapsed={elapsed:.1f}s)")
                    
                    self.add_gaussian_input(object_name)
                    self.picked_up_objects.add(object_name)
            
        except Exception as e:
            rospy.logerr(f"Error processing detection message: {e}")


    def publish_slices(self, event):
        """
        Publish combined input matrices for the current timestep.
        Robot feedback and human voice inputs persist indefinitely once activated.
        """
        if self.start_time is None:
            self.start_time = time()
        
        if self.current_time_index < len(self.t):
            self.update_input_matrices() # Updates the vision input
            
            # Use accumulated inputs (these persist forever once set)
            robot_input_for_this_step = self.accumulated_robot_feedback
            human_input_for_this_step = self.accumulated_human_voice

            # Build the combined message
            combined_input = [
                self.input_matrix1[self.current_time_index].tolist(),  # Slot 1: Vision
                robot_input_for_this_step.tolist(),                   # Slot 2: Robot Feedback
                human_input_for_this_step.tolist()                    # Slot 3: Human Voice
            ]

            # Create and publish the message
            msg = Float32MultiArray()
            msg.data = [item for sublist in combined_input for item in sublist]
            self.pub_inputs.publish(msg)

            # Logging
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
            rospy.loginfo(f"Completed publishing all time slices.")
            self.main_publishing_timer.shutdown()


def main():
    try:
        VisionToDNF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()