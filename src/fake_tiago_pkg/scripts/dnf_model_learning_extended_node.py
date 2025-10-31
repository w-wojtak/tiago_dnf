#!/usr/bin/env python3
import rospy
import numpy as np
import os
import rospkg
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from datetime import datetime
import threading


class DNFLearningNode:
    def __init__(self):
        rospy.init_node('dnf_model_learning_simple_node', anonymous=True)

        # Simulation parameters
        self.x_lim, self.t_lim, self.dx, self.dt = 80, 15, 0.2, 0.1
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.t = np.arange(0, self.t_lim + self.dt, self.dt)
        self.input_positions = [-60, -20, 20, 40]

        self.input_indices_sm = [np.argmin(np.abs(self.x - pos)) for pos in self.input_positions]
        self.center_index = int(len(self.x) / 2)

        # --- Duplicated History Lists ---
        self.u_sm_1_history = []
        self.u_sm_2_history = []
        self.u_d_history = []

        # --- Sequence memory field 1 ---
        self.h_0_sm_1, self.tau_h_sm_1, self.theta_sm_1 = 0, 20, 1.5
        self.kernel_pars_sm_1 = (1, 0.7, 0.9)
        self.w_hat_sm_1 = np.fft.fft(self.kernel_osc(*self.kernel_pars_sm_1))
        self.u_sm_1 = self.h_0_sm_1 * np.ones_like(self.x)
        self.h_u_sm_1 = self.h_0_sm_1 * np.ones_like(self.x)

        # --- Sequence memory field 2 ---
        self.h_0_sm_2, self.tau_h_sm_2, self.theta_sm_2 = 0, 20, 1.5
        self.kernel_pars_sm_2 = (1, 0.7, 0.9)
        self.w_hat_sm_2 = np.fft.fft(self.kernel_osc(*self.kernel_pars_sm_2))
        self.u_sm_2 = self.h_0_sm_2 * np.ones_like(self.x)
        self.h_u_sm_2 = self.h_0_sm_2 * np.ones_like(self.x)

        # Detection field
        self.h_0_d, self.tau_h_d, self.theta_d = 0, 20, 1.5
        self.kernel_pars_d = (1, 0.7, 0.9)
        self.w_hat_d = np.fft.fft(self.kernel_osc(*self.kernel_pars_d))
        self.u_d = self.h_0_d * np.ones_like(self.x)
        self.h_u_d = self.h_0_d * np.ones_like(self.x)

        self.time_counter = 0.0
        self.current_step = 1
        
        # Thread lock for data safety
        self.data_lock = threading.Lock()
        self.plot_needs_update = False

        # --- Updated Interactive Plot ---
        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 5))
        self.line1, = self.ax1.plot(self.x, self.u_sm_1, 'b-', label='u_sm_1', linewidth=2)
        self.line2, = self.ax2.plot(self.x, self.u_sm_2, 'g-', label='u_sm_2', linewidth=2)
        self.line3, = self.ax3.plot(self.x, self.u_d, 'r-', label='u_d', linewidth=2)
        
        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()
        self.setup_axes()
        self.fig.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)

        self.shutdown_requested = False
        self.is_finished = False

        rospy.loginfo("Plot window initialized")

        # ROS subscriber
        self.sub = rospy.Subscriber('/dnf_inputs', Float32MultiArray, 
                                   self.input_callback, queue_size=10)
        
        rospy.loginfo("DNF Learning Node initialized and ready")

    def input_callback(self, msg):
        """Process incoming data - runs in ROS callback thread"""
        data = np.array(msg.data)
        n = len(data) // 3
        input1, _, _ = data[:n], data[n:2*n], data[2*n:]

        self.time_counter += self.dt
        input_d = self.gaussian(0, 5.0, 2.0) if self.time_counter < 1.0 else np.zeros_like(self.x)

        # Thread-safe data update
        with self.data_lock:
            # Update detection field
            f_d = np.heaviside(self.u_d - self.theta_d, 1)
            conv_d = self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f_d) * self.w_hat_d)))
            self.h_u_d += self.dt / self.tau_h_d * f_d
            self.u_d += self.dt * (-self.u_d + conv_d + input_d + self.h_u_d)

            # --- Update sequence memory field 1 ---
            f_sm_1 = np.heaviside(self.u_sm_1 - self.theta_sm_1, 1)
            conv_sm_1 = self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f_sm_1) * self.w_hat_sm_1)))
            self.h_u_sm_1 += self.dt / self.tau_h_sm_1 * f_sm_1
            self.u_sm_1 += self.dt * (-self.u_sm_1 + conv_sm_1 + input1 + self.h_u_sm_1)

            # --- Update sequence memory field 2 ---
            f_sm_2 = np.heaviside(self.u_sm_2 - self.theta_sm_2, 1)
            conv_sm_2 = self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f_sm_2) * self.w_hat_sm_2)))
            self.h_u_sm_2 += self.dt / self.tau_h_sm_2 * f_sm_2
            self.u_sm_2 += self.dt * (-self.u_sm_2 + conv_sm_2 + input1 + self.h_u_sm_2)

            # --- Track values over time ---
            u_sm_1_values = [self.u_sm_1[idx] for idx in self.input_indices_sm]
            u_sm_2_values = [self.u_sm_2[idx] for idx in self.input_indices_sm]
            u_d_value = self.u_d[self.center_index]
            self.u_sm_1_history.append(u_sm_1_values)
            self.u_sm_2_history.append(u_sm_2_values)
            self.u_d_history.append(u_d_value)

            # Signal that plot needs update
            self.plot_needs_update = True

        # Logging
        if int(self.time_counter) >= self.current_step:
            rospy.loginfo(f"Time {self.current_step:.1f}s | "
                         f"u_sm_1: [{self.u_sm_1.min():.2f}, {self.u_sm_1.max():.2f}] | "
                         f"u_sm_2: [{self.u_sm_2.min():.2f}, {self.u_sm_2.max():.2f}] | "
                         f"u_d: [{self.u_d.min():.2f}, {self.u_d.max():.2f}]")
            self.current_step += 1

        # Check if finished
        if self.time_counter >= self.t_lim:
            if not self.is_finished:
                rospy.loginfo("Learning finished. Signaling main loop to save and exit.")
                self.is_finished = True

    def update_plot(self):
        """Update plot - must be called from main thread"""
        if not self.plot_needs_update:
            return
            
        try:
            with self.data_lock:
                # Copy data for plotting
                u_sm_1_copy = self.u_sm_1.copy()
                u_sm_2_copy = self.u_sm_2.copy()
                u_d_copy = self.u_d.copy()
                self.plot_needs_update = False
            
            # --- Update plot data ---
            self.line1.set_ydata(u_sm_1_copy)
            self.line2.set_ydata(u_sm_2_copy)
            self.line3.set_ydata(u_d_copy)
            
            # Rescale if needed
            for ax in [self.ax1, self.ax2, self.ax3]:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
            
            # Trigger redraw
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            rospy.logwarn(f"Plot update failed: {e}")

    def spin(self):
        """Custom spin that keeps matplotlib responsive"""
        rate = rospy.Rate(20)  # 20 Hz update rate
        
        while not rospy.is_shutdown():
            if self.is_finished:
                rospy.loginfo("Main loop detected 'finished' flag. Saving data and shutting down.")
                self.save_data()
                break

            try:
                self.update_plot()
                plt.pause(0.001)
                rate.sleep()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                break

    def kernel_osc(self, a, b, alpha):
        return a * (np.exp(-b * np.abs(self.x)) * 
                   (b * np.sin(np.abs(alpha * self.x)) + np.cos(alpha * self.x)))

    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        return amplitude * np.exp(-((self.x - center)**2) / (2*width**2))

    def setup_axes(self):
        object_all = [-60, -40, -20, 0, 20, 40, 60]
        object_labels = ['base', 'blue box', 'load', 'tool 1', 'bearing', 'motor', 'tool 2']

        # Apply settings to all three axes
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_xlim(-self.x_lim, self.x_lim)
            ax.set_ylim(-2, 6)
            ax.set_xlabel("Objects", fontsize=10)
            ax.set_ylabel("u(x)", fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_xticks(object_all)
            ax.set_xticklabels(object_labels, rotation=45, ha='right', fontsize=9)
            for pos in object_all:
                ax.axvline(x=pos, color='gray', linestyle='--', alpha=0.3, linewidth=0.5)

        self.ax1.set_title("Sequence Memory 1", fontsize=12, fontweight='bold')
        self.ax2.set_title("Sequence Memory 2", fontsize=12, fontweight='bold')
        self.ax3.set_title("Task Duration Field", fontsize=12, fontweight='bold')

    def save_data(self):
        self.shutdown_requested = True
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('fake_tiago_pkg')
            data_dir = os.path.join(pkg_path, "data_extended")
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            with self.data_lock:
                # --- Save both u_sm fields ---
                np.save(os.path.join(data_dir, f"u_sm_1_{timestamp}.npy"), self.u_sm_1)
                np.save(os.path.join(data_dir, f"u_sm_2_{timestamp}.npy"), self.u_sm_2)
                np.save(os.path.join(data_dir, f"u_d_{timestamp}.npy"), self.u_d)

            # --- Convert lists to arrays ---
            u_sm_1_history = np.array(self.u_sm_1_history)
            u_sm_2_history = np.array(self.u_sm_2_history)
            u_d_history = np.array(self.u_d_history)
            timesteps = np.arange(len(u_sm_1_history)) * self.dt

            # --- Compute crossing times for both sm fields ---
            for i, pos in enumerate(self.input_positions):
                above_1 = u_sm_1_history[:, i] >= self.theta_sm_1
                if np.any(above_1):
                    crossing_idx_1 = np.argmax(above_1)
                    rospy.loginfo(f"u_sm_1 at x={pos} crosses theta at time {crossing_idx_1 * self.dt:.2f}s")
                
                above_2 = u_sm_2_history[:, i] >= self.theta_sm_2
                if np.any(above_2):
                    crossing_idx_2 = np.argmax(above_2)
                    rospy.loginfo(f"u_sm_2 at x={pos} crosses theta at time {crossing_idx_2 * self.dt:.2f}s")

            above_d = u_d_history >= self.theta_d
            if np.any(above_d):
                crossing_idx_d = np.argmax(above_d)
                rospy.loginfo(f"u_d at x=0 crosses theta at time {crossing_idx_d * self.dt:.2f}s")

            # --- Plot histories (now with 3 subplots) ---
            fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

            # Plot u_sm_1
            for i, pos in enumerate(self.input_positions):
                axs[0].plot(timesteps, u_sm_1_history[:, i], label=f'x = {pos}')
            axs[0].axhline(self.theta_sm_1, color='r', linestyle='--', label=f'theta = {self.theta_sm_1}')
            axs[0].set_ylabel('u_sm_1')
            axs[0].set_ylim(-1, 5)
            axs[0].legend()
            axs[0].grid(True)
            axs[0].set_title("u_sm_1 History")

            # Plot u_sm_2
            for i, pos in enumerate(self.input_positions):
                axs[1].plot(timesteps, u_sm_2_history[:, i], label=f'x = {pos}')
            axs[1].axhline(self.theta_sm_2, color='g', linestyle='--', label=f'theta = {self.theta_sm_2}')
            axs[1].set_ylabel('u_sm_2')
            axs[1].set_ylim(-1, 5)
            axs[1].legend()
            axs[1].grid(True)
            axs[1].set_title("u_sm_2 History")

            # Plot u_d
            axs[2].plot(timesteps, u_d_history, label='x = 0')
            axs[2].axhline(self.theta_d, color='r', linestyle='--', label=f'theta = {self.theta_d}')
            axs[2].set_ylabel('u_d')
            axs[2].set_xlabel('Time [s]')
            axs[2].set_ylim(0, 5)
            axs[2].legend()
            axs[2].grid(True)
            axs[2].set_title("u_d History")
            
            fig.tight_layout()

            # Save the time-course figure
            timecourse_path = os.path.join(data_dir, f"timecourse_{timestamp}.png")
            fig.savefig(timecourse_path, dpi=150, bbox_inches='tight')
            plt.close(fig)

            rospy.loginfo(f"Saved time-course plot at {timecourse_path}")

            
            # Save final plot
            self.fig.savefig(os.path.join(data_dir, f"final_plot_{timestamp}.png"), 
                           dpi=150, bbox_inches='tight')
            
            rospy.loginfo(f"Saved data and plot in {data_dir}")
        except Exception as e:
            rospy.logerr(f"Error saving data: {e}")


if __name__ == "__main__":
    try:
        node = DNFLearningNode()
        rospy.loginfo("DNF Learning Node started. Waiting for inputs...")
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.close('all')