#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import re
import numpy as np
import matplotlib.pyplot as plt
import threading
from utils import *
import os
from scipy.ndimage import gaussian_filter1d 
from datetime import datetime

class DNFRecallNode:
    def __init__(self):
        rospy.init_node("dnf_recall_node", anonymous=True)

        # --- Parameters ---
        self.trial_number = rospy.get_param('~trial_number', 1)
        rospy.loginfo(f"****** Recall node ****** started with trial_number: {self.trial_number}")

        # Spatial and temporal grid
        self.x_lim, self.t_lim = 80, 15
        self.dx, self.dt = 0.2, 0.1
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        rospy.loginfo(f"Spatial grid size: {len(self.x)}")

        self.h_u_amem_current_trial  = np.zeros(np.shape(self.x))

        self.input_positions = [-60, -20, 20, 40]
        self.input_indices = [np.argmin(np.abs(self.x - pos)) for pos in self.input_positions]

        # Object mapping
        object_positions = [-60, -40, -20, 0, 20, 40, 60]
        object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']

        # Keep only relevant ones
        self.object_map = {
            pos: label for pos, label in zip(object_positions, object_labels)
            if label in ['base', 'load', 'bearing', 'motor']
        }

        self.u_sm_history = []  # list of lists (each timestep)
        self.u_d_history = []

        # Threading lock for concurrency
        self.data_lock = threading.Lock()

        # --- Timing ---
        self.start_time = rospy.Time.now()
        self.current_sim_step = 0
        rospy.loginfo("Recall node initialized. Timer started.")

        # Publisher for threshold crossings
        # self.publisher = rospy.Publisher('threshold_crossings', Float32MultiArray, queue_size=10)
        self.publisher = rospy.Publisher('threshold_crossings', String, queue_size=10)

        self.response_pub = rospy.Publisher('/response_command', String, queue_size=5)

        # --- Plotting ---
        self._init_plots()

        # --- Load previous data ---
        # Get paths from ROS parameters
        self.base_data_path = rospy.get_param('~base_data_path', '/workspaces/fake_tiago_ws/src/fake_tiago_pkg/data_extended')
        self.trial_data_path = rospy.get_param('~trial_data_path', f'{self.base_data_path}/trial_{self.trial_number}')
        
        rospy.loginfo(f"Base data path (u_sm, u_d): {self.base_data_path}")
        rospy.loginfo(f"Trial data path (h_u_amem): {self.trial_data_path}")
        
        self._load_data(self.base_data_path)

        # cooldown to prevent repeating the same message too often
        self.error_last_sent = {pos: 0.0 for pos in self.input_positions}
        self.error_cooldown = 100.0  # seconds between messages per object

        # --- Field initialization ---
        self._init_fields()

        # --- History containers ---
        self.u_act_history, self.u_sim_history = [], []
        self.u_wm_history, self.u_f1_history, self.u_f2_history, self.u_error_history = [], [], [], []

        self.error_threshold_crossed = {pos: False for pos in self.input_positions}

        # --- Subscribers and timers ---
        self.subscription = rospy.Subscriber(
            '/dnf_inputs',
            Float32MultiArray,
            self.process_inputs,
            queue_size=10
        )
        
        # Register shutdown hook
        rospy.on_shutdown(self.save_all_data)


    def sanitize_for_tts(self, text):
        """Keep the message clean and short for TTS."""
        text = re.sub(r"[^A-Za-z0-9 ,.'!?-]", "", text)
        text = re.sub(r"\s+", " ", text).strip()
        return text[:200]

    # ------------------ Setup helpers ------------------
    def _init_plots(self):
        plt.ion()
        self.fig, axes = plt.subplots(2, 3, figsize=(15, 9))
        self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6 = axes.flatten()

        object_positions = [-60, -40, -20, 0, 20, 40, 60]
        object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']

        # Define the desired limits once
        desired_xlim = (self.x.min(), self.x.max()) # e.g., (-80, 80)

        # Pass the new xlim argument to each call
        self.ax1 = format_axis(self.ax1, "Action Onset Field", "u_act(x)", object_positions, object_labels, xlim=desired_xlim)
        self.ax2 = format_axis(self.ax2, "Simulation Field", "u_sim(x)", object_positions, object_labels, xlim=desired_xlim)
        self.ax3 = format_axis(self.ax3, "Working Memory Field", "u_wm(x)", object_positions, object_labels, xlim=desired_xlim)
        self.ax4 = format_axis(self.ax4, "Feedback 1 Field", "u_f1(x)", object_positions, object_labels, xlim=desired_xlim)
        self.ax5 = format_axis(self.ax5, "Feedback 2 Field", "u_f2(x)", object_positions, object_labels, xlim=desired_xlim)
        self.ax6 = format_axis(self.ax6, "Error Field", "u_error(x)", object_positions, object_labels, xlim=desired_xlim)

        # Initialize line objects
        self.line_act, = self.ax1.plot(self.x, np.zeros_like(self.x), label="u_act")
        self.line_sim, = self.ax2.plot(self.x, np.zeros_like(self.x), label="u_sim")
        self.line_wm, = self.ax3.plot(self.x, np.zeros_like(self.x), label="u_wm")
        self.line_f1, = self.ax4.plot(self.x, np.zeros_like(self.x), label="u_f1")
        self.line_f2, = self.ax5.plot(self.x, np.zeros_like(self.x), label="u_f2")
        self.line_error, = self.ax6.plot(self.x, np.zeros_like(self.x), label="u_error")

        plt.tight_layout()
        plt.show(block=False)

    def _load_data(self, data_dir):
        """Load sequence memory and task duration from disk (same for all trials)."""
        self.h_d_initial = 0
        try:
            self.u_d = load_task_duration(data_dir)
            self.h_d_initial = max(self.u_d)
            rospy.loginfo(f"✓ Loaded task duration from {data_dir}, size: {len(self.u_d)}, max value: {self.h_d_initial:.3f}")

            # Load adaptive memory from previous trial (or initialize to zeros)
            self.h_u_amem = self._load_adaptive_memory()

            # Load sequence memory
            # u_sm = load_sequence_memory(data_dir)
            u_sm_1, u_sm_2 = load_sequence_memory_extended(data_dir)
            self.u_act = u_sm_1 - self.h_d_initial + 1.525 - self.h_u_amem
            self.u_sim = u_sm_2 - self.h_d_initial + 1.525 - self.h_u_amem
            self.input_action_onset = u_sm_1.copy()
            self.input_action_onset_2 = u_sm_2.copy()
            rospy.loginfo(f"✓ Loaded sequence memory from {data_dir}")

        except IOError as e:
            rospy.logwarn(f"✗ No previous sequence memory found in {data_dir}: {e}")
            self.u_act = np.zeros_like(self.x)
            self.u_sim = np.zeros_like(self.x)
            self.input_action_onset = np.zeros_like(self.x)
            self.input_action_onset_2 = np.zeros_like(self.x)


    def _load_adaptive_memory(self):
        """Load h_u_amem from previous trial, or initialize to zeros."""
        if self.trial_number == 1:
            # First trial - start fresh
            rospy.loginfo("Trial 1: Initializing h_u_amem to zeros")
            return np.zeros_like(self.x)
        else:
            # Load from previous trial
            prev_trial = self.trial_number - 1
            
            # Construct path to previous trial's h_u_amem
            prev_trial_path = os.path.join(self.base_data_path, f'trial_{prev_trial}')
            filepath = os.path.join(prev_trial_path, 'h_u_amem.npy')
            
            if os.path.exists(filepath):
                loaded_amem = np.load(filepath)
                rospy.loginfo(f" Loaded h_u_amem from trial {prev_trial}")
                rospy.loginfo(f"  Path: {filepath}")
                rospy.loginfo(f"  Shape: {loaded_amem.shape}, Max: {np.max(loaded_amem):.4f}, Min: {np.min(loaded_amem):.4f}")
                return loaded_amem
            else:
                rospy.logwarn(f"✗ No h_u_amem found at {filepath}. Starting fresh.")
                return np.zeros_like(self.x)

    def _init_fields(self):
        """Initialize all fields and parameters."""
        self.u_wm = -1.0 * np.ones_like(self.x)
        self.u_f1 = -1.0 * np.ones_like(self.x)
        self.u_f2 = -1.0 * np.ones_like(self.x)
        self.u_error = -1.0 * np.ones_like(self.x)

        # Thresholds
        self.theta_act = 1.5
        self.theta_sim = 1.5
        self.theta_wm = 0.8
        self.theta_error = 1.5
        self.theta_f = 1.5

        # Decay rates
        self.tau_h_act = 20
        self.tau_h_sim = 10
        self.beta_adapt = 0.0015

        # Kernels (precompute FFTs)
        self.w_hat_act = np.fft.fft(kernel_gauss(self.x, 1.5, 0.8, 0.0))
        self.w_hat_sim = np.fft.fft(kernel_gauss(self.x, 1.7, 0.8, 1.0))
        self.w_hat_wm = np.fft.fft(kernel_osc(self.x, 1.75, 0.5, 0.8))
        self.w_hat_f = self.w_hat_act.copy()

        # Load adaptive memory from previous trial (or initialize to zeros)
        self.h_u_amem = self._load_adaptive_memory()

        # Adaptation fields
        self.h_u_act = -self.h_d_initial * np.ones_like(self.x) + 1.525 - self.h_u_amem
        self.h_u_sim = -self.h_d_initial * np.ones_like(self.x) + 1.525 - self.h_u_amem
        self.h_u_wm = -1.0 * np.ones_like(self.x)
        

        
        self.h_f = -1.0

        # Inputs from subscription
        self.input_vision = np.zeros_like(self.x)       # From vision_to_dnf
        self.input_robot_feedback = np.zeros_like(self.x) # For u_f1
        self.input_human_voice = np.zeros_like(self.x)    # For u_f2

        # Threshold tracker
        self.threshold_crossed = {pos: False for pos in [-60, -20, 20, 40]}


    def get_closest_object_label(self, pos):
        object_positions = [-60, -40, -20, 0, 20, 40, 60]
        object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']
        idx = np.argmin(np.abs(np.array(object_positions) - pos))
        return object_labels[idx]

    # ------------------ ROS Callbacks ------------------
    def process_inputs(self, msg):
        """
        Process the combined input matrices from the '/dnf_inputs' topic.
        Unpacks the data according to the established contract.
        """
        try:
            data = np.array(msg.data)
            n = len(self.x)

            if len(data) != 3 * n:
                rospy.logwarn(f"Received data of unexpected length. Skipping.")
                return

            with self.data_lock:
                # --- UNPACK DATA ---
                # Matrix 1: Vision Input
                self.input_vision = data[0 : n]
                
                # Matrix 2: Robot Feedback Input (for u_f1)
                self.input_robot_feedback = data[n : 2*n]
                
                # Matrix 3: Human Voice Input (for u_f2)
                self.input_human_voice = data[2*n : 3*n]
                # --- END OF UNPACKING ---

            # Now that inputs are updated, run one step of the simulation.
            self.perform_recall()

        except Exception as e:
            rospy.logerr(f"Error in process_inputs: {e}")

    # ------------------ Core Recall ------------------
    def perform_recall(self):
        with self.data_lock:
            # Increment simulation time step and calculate current times
            self.current_sim_step += 1
            elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
            current_sim_time = self.current_sim_step * self.dt
            # --- Compute convolutions using FFTs ---
            def conv(field, w_hat, theta):
                f = np.heaviside(field - theta, 1)
                return self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f) * w_hat)))


            conv_act = conv(self.u_act, self.w_hat_act, self.theta_act)
            conv_sim = conv(self.u_sim, self.w_hat_sim, self.theta_sim)
            conv_wm = conv(self.u_wm, self.w_hat_wm, self.theta_wm)
            conv_f1 = conv(self.u_f1, self.w_hat_f, self.theta_f)
            conv_f2 = conv(self.u_f2, self.w_hat_f, self.theta_f)
            conv_error = conv(self.u_error, self.w_hat_act, self.theta_error)
            # conv_error = conv(self.u_error, self.w_hat_wm, self.theta_error)

            f_wm = np.heaviside(self.u_wm - self.theta_wm, 1)
            f_sim = np.heaviside(self.u_sim - self.theta_sim, 1)
            f_uf2 = np.heaviside(self.u_f2 - self.theta_f, 1)

            # --- Update field dynamics ---
            self.h_u_act += self.dt / self.tau_h_act
            self.h_u_sim += self.dt / self.tau_h_sim

            self.u_act += self.dt * (-self.u_act + conv_act + self.input_action_onset + self.h_u_act - 6.0 * f_wm * conv_wm)
            self.u_sim += (self.dt*3.0) * (-self.u_sim + conv_sim + self.input_action_onset_2 + self.h_u_sim - 6.0 * f_wm *conv_wm)
            self.u_wm += self.dt * (-self.u_wm + conv_wm + 6*((conv_f1*self.u_f1)*(conv_f2*self.u_f2)) + self.h_u_wm)
            self.u_f1 += self.dt * (-self.u_f1 + conv_f1 + self.input_robot_feedback + self.h_f - 2*conv_wm)
            self.u_f2 += self.dt * (-self.u_f2 + conv_f2 + self.input_human_voice + self.h_f - 2*conv_wm)
            self.u_error += self.dt * (-self.u_error + conv_error + self.h_f - 4*conv_sim*f_sim + (0.8*self.u_f2*f_uf2))

            # Update adaptive memory (this will be saved for next trial)
            # self.h_u_amem += self.beta_adapt*(1 - (conv_f2*conv_f1))*(conv_f1 - conv_f2)
            self.h_u_amem_current_trial += self.beta_adapt*(1 - (conv_f2*conv_f1))*(conv_f1 - conv_f2)

            # --- Threshold detection and history ---


            self.u_act_history.append([self.u_act[idx] for idx in self.input_indices])
            self.u_sim_history.append([self.u_sim[idx] for idx in self.input_indices])
            self.u_wm_history.append([self.u_wm[idx] for idx in self.input_indices])
            self.u_f1_history.append([self.u_f1[idx] for idx in self.input_indices])
            self.u_f2_history.append([self.u_f2[idx] for idx in self.input_indices])
            self.u_error_history.append([self.u_error[idx] for idx in self.input_indices])


            for i, idx in enumerate(self.input_indices):
                pos = self.input_positions[i]
                if not self.threshold_crossed[pos] and self.u_act[idx] > self.theta_act:
                    rospy.loginfo(
                    f"[{elapsed_time:.2f}s | Sim Time: {current_sim_time:.1f}s] "
                    f"***** PREDICTION: Threshold crossed at position {pos} (u_act={self.u_act[idx]:.2f})"
                    )
                    # msg = Float32MultiArray()
                    # msg.data = [float(pos)]
                    # self.publisher.publish(msg)
                    # self.threshold_crossed[pos] = True

                    label = self.object_map.get(pos, str(pos))
                    msg = String(data=label)
                    self.publisher.publish(msg)
                    self.threshold_crossed[pos] = True
                    rospy.loginfo(f"***** Published threshold crossing for object: {label}")


            # # Map input_positions to object labels
            object_positions = [-60, -40, -20, 0, 20, 40, 60]
            object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']

            # # Function to get closest object label
            # def get_closest_object_label(pos):
            #     idx = np.argmin(np.abs(np.array(object_positions) - pos))
            #     return object_labels[idx]

            # def get_closest_object_label(self, pos):
            #     object_positions = [-60, -40, -20, 0, 20, 40, 60]
            #     object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']
            #     idx = np.argmin(np.abs(np.array(object_positions) - pos))
            #     return object_labels[idx]


            # --- u_error threshold detection with correct object from u_sim ---
            for i, idx in enumerate(self.input_indices):
                pos = self.input_positions[i]
                if not self.error_threshold_crossed[pos] and self.u_error[idx] > self.theta_error:
                    # Determine correct object from u_sim
                    correct_idx = np.argmax(self.u_sim)
                    correct_pos = self.x[correct_idx]
                    object_name = None
                    for label, obj_pos in zip(object_labels, object_positions):
                        if np.isclose(obj_pos, correct_pos, atol=2.0):
                            object_name = label
                            break

                    wrong_obj = self.get_closest_object_label(pos)

                    msg_text = f"ERROR: '{wrong_obj}' is not correct, you should use '{object_name}'"
                    rospy.loginfo(msg_text)

                    # --- Publish to /response_command ---
                    ros_msg = String()
                    ros_msg.data = msg_text
                    self.response_pub.publish(ros_msg)

                    # Mark as sent
                    self.error_threshold_crossed[pos] = True



    # ------------------ Plotting ------------------
    def update_plot(self):
        try:
            with self.data_lock:
                self.line_act.set_ydata(self.u_act)
                self.line_sim.set_ydata(self.u_sim)
                self.line_wm.set_ydata(self.u_wm)
                self.line_f1.set_ydata(self.u_f1)
                self.line_f2.set_ydata(self.u_f2)
                self.line_error.set_ydata(self.u_error)
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
        except Exception as e:
            rospy.logerr(f"Plot update error: {e}")

    # ------------------ Save data ------------------
    def save_all_data(self):
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        rospy.loginfo(f"[{elapsed_time:.2f}s] Shutting down. Saving field data.")
        
        # Save working memory (existing functionality)
        save_field(self.u_wm, "working_memory")
        save_node_history(self)
        
        # Save adaptive memory for next trial
        self._save_adaptive_memory()

        # Save time-course plot
        self.save_timecourse_plot()

    def _save_adaptive_memory(self):
        """Save h_u_amem for use in the next trial (smoothed and accumulated)."""
        try:
            # Create trial-specific directory if it doesn't exist
            if not os.path.exists(self.trial_data_path):
                os.makedirs(self.trial_data_path)
                rospy.loginfo(f"Created directory: {self.trial_data_path}")
            
            # Apply Gaussian smoothing
            # h_u_amem_smoothed = gaussian_filter1d(self.h_u_amem, sigma=10)
            h_u_amem_smoothed = gaussian_filter1d(self.h_u_amem_current_trial, sigma=10)
            
            
            # Accumulate with previous trial if not trial 1
            if self.trial_number > 1:
                prev_trial = self.trial_number - 1
                prev_trial_path = os.path.join(self.base_data_path, f'trial_{prev_trial}')
                prev_filepath = os.path.join(prev_trial_path, 'h_u_amem.npy')
                
                if os.path.exists(prev_filepath):
                    latest_h_amem = np.load(prev_filepath)
                    h_u_amem_smoothed = h_u_amem_smoothed + latest_h_amem
                    rospy.loginfo(f"  Accumulated with trial {prev_trial} h_u_amem")
                else:
                    rospy.logwarn(f"  Could not find previous h_u_amem at {prev_filepath}")
            
            # Get current timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save to trial-specific directory
            filepath = os.path.join(self.trial_data_path, 'h_u_amem.npy')
            np.save(filepath, h_u_amem_smoothed)
            
            # Also save with timestamp for archival
            filepath_timestamped = os.path.join(self.trial_data_path, f'h_u_amem_{timestamp}.npy')
            np.save(filepath_timestamped, h_u_amem_smoothed)
            
            rospy.loginfo(f"✓ Saved h_u_amem for trial {self.trial_number} (smoothed with sigma=15)")
            rospy.loginfo(f"  Path: {filepath}")
            rospy.loginfo(f"  Timestamped: {filepath_timestamped}")
            rospy.loginfo(f"  Original - Max: {np.max(self.h_u_amem):.4f}, Min: {np.min(self.h_u_amem):.4f}")
            rospy.loginfo(f"  Saved (smoothed+accumulated) - Max: {np.max(h_u_amem_smoothed):.4f}, Min: {np.min(h_u_amem_smoothed):.4f}")
            
        except Exception as e:
            rospy.logerr(f"✗ Failed to save h_u_amem: {e}")


    def save_timecourse_plot(self):
        """Save time-course plots of u_f1, u_f2, u_act, u_sim at input positions."""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            data_dir = self.trial_data_path
            os.makedirs(data_dir, exist_ok=True)

            # Convert histories to arrays
            u_f1_history = np.array(self.u_f1_history)
            u_f2_history = np.array(self.u_f2_history)
            u_act_history = np.array(self.u_act_history)
            u_sim_history = np.array(self.u_sim_history)
            u_error_history = np.array(self.u_error_history)
            timesteps = np.arange(len(u_f1_history)) * self.dt

            theta_f = 1.5
            theta_act = self.theta_act
            theta_sim = self.theta_sim
            theta_error = self.theta_error

            # Check crossings and store them
            crossings = {}
            for i, pos in enumerate(self.input_positions):
                crossings[pos] = {}
                above_f1 = u_f1_history[:, i] >= theta_f
                crossings[pos]['u_f1'] = (np.argmax(above_f1) * self.dt) if np.any(above_f1) else None

                above_f2 = u_f2_history[:, i] >= theta_f
                crossings[pos]['u_f2'] = (np.argmax(above_f2) * self.dt) if np.any(above_f2) else None

                above_act = u_act_history[:, i] >= theta_act
                crossings[pos]['u_act'] = (np.argmax(above_act) * self.dt) if np.any(above_act) else None

                above_sim = u_sim_history[:, i] >= theta_sim
                crossings[pos]['u_sim'] = (np.argmax(above_sim) * self.dt) if np.any(above_sim) else None

                # Log the crossings
                for field_name, cross_time in crossings[pos].items():
                    if cross_time is not None:
                        rospy.loginfo(f"{field_name} at x={pos} crosses theta at {cross_time:.2f}s")


            rospy.loginfo("--- Threshold Crossing Summary ---")

            # Define the fields you want to summarize
            field_names_to_summarize = ['u_act', 'u_sim', 'u_f1', 'u_f2']

            # Restructure the data from being grouped by position to being grouped by field
            summary_crossings = {name: [] for name in field_names_to_summarize}

            for pos in self.input_positions: # Iterate in order of positions
                for field_name in field_names_to_summarize:
                    cross_time = crossings[pos].get(field_name)
                    if cross_time is not None:
                        summary_crossings[field_name].append(cross_time)
            
            # Print the summary in the desired format
            for field_name, times_list in summary_crossings.items():
                # Format each time to ".2f" and join with spaces
                times_str = " ".join([f"{t:.2f}" for t in times_list])
                rospy.loginfo(f"{field_name} threshold crossings: {times_str}")

            # --- Plot time-courses (5 panels vertically stacked) ---
            fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
            field_data = [
                (u_act_history, theta_act, "u_act"),
                (u_sim_history, theta_sim, "u_sim"),
                (u_f1_history, theta_f, "u_f1 (robot)"),
                (u_f2_history, theta_f, "u_f2 (human)"),
                (u_error_history, theta_error, "u_error"),
            ]

            for ax, (history, theta, label) in zip(axes, field_data):
                for i, pos in enumerate(self.input_positions):
                    ax.plot(timesteps, history[:, i], label=f"x={pos}")
                ax.axhline(theta, color="r", linestyle="--", label=f"θ={theta}")
                ax.set_ylabel(label)
                ax.grid(True)
                ax.legend()

            ax1, ax2, ax3, ax4, ax5 = axes

            ax1.set_ylim(-5, 5)
            ax2.set_ylim(-5, 5)
            ax3.set_ylim(-5, 5)
            ax4.set_ylim(-5, 5)
            ax5.set_ylim(-5, 5)

            axes[-1].set_xlabel("Time (s)")
            fig.suptitle("Field Activity Over Time", fontsize=14)
            fig.tight_layout(rect=[0, 0, 1, 0.97])

            # Save the figure
            fig_path = os.path.join(
                data_dir, f"timecourse_trial{self.trial_number}_{timestamp}.png"
            )
            fig.savefig(fig_path)
            plt.close(fig)
            rospy.loginfo(f"✓ Time-course plot saved: {fig_path}")

            # --- Save data in compressed file ---
            npz_path = os.path.join(
                data_dir, f"dnf_recall_results_trial{self.trial_number}_{timestamp}.npz"
            )
            np.savez_compressed(
                npz_path,
                timesteps=timesteps,
                u_act_history=u_act_history,
                u_sim_history=u_sim_history,
                u_f1_history=u_f1_history,
                u_f2_history=u_f2_history,
                u_error_history=u_error_history,
                input_positions=np.array(self.input_positions),
                threshold_crossings=crossings
            )
            rospy.loginfo(f"✓ Data saved: {npz_path}")

        except Exception as e:
            rospy.logerr(f"✗ Failed to save time-course plot/data: {e}")

            



# ------------------ Main ------------------
def main():
    node = DNFRecallNode()
    rospy.loginfo("DNF Recall Node started. Waiting for input...")

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            node.update_plot()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.save_all_data()
        plt.close('all')

if __name__ == "__main__":
    main()