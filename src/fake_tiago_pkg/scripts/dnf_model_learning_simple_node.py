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

        # Sequence memory field
        self.h_0_sm, self.tau_h_sm, self.theta_sm = 0, 20, 1.5
        self.kernel_pars_sm = (1, 0.7, 0.9)
        self.w_hat_sm = np.fft.fft(self.kernel_osc(*self.kernel_pars_sm))
        self.u_sm = self.h_0_sm * np.ones_like(self.x)
        self.h_u_sm = self.h_0_sm * np.ones_like(self.x)

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

        # Initialize interactive plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        self.line1, = self.ax1.plot(self.x, self.u_sm, 'b-', label='u_sm', linewidth=2)
        self.line2, = self.ax2.plot(self.x, self.u_d, 'r-', label='u_d', linewidth=2)
        
        self.ax1.legend()
        self.ax2.legend()
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
        input1, input2, _ = data[:n], data[n:2*n], data[2*n:]

        self.time_counter += self.dt
        input_d = self.gaussian(0, 5.0, 2.0) if self.time_counter < 1.0 else np.zeros_like(self.x)

        # Thread-safe data update
        with self.data_lock:
            # Update detection field
            f_d = np.heaviside(self.u_d - self.theta_d, 1)
            # conv_d = self.dx * np.real(np.fft.ifft(np.fft.fft(f_d) * self.w_hat_d))
            conv_d = self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f_d) * self.w_hat_d)))
            self.h_u_d += self.dt / self.tau_h_d * f_d
            self.u_d += self.dt * (-self.u_d + conv_d + input_d + self.h_u_d)

            # Update sequence memory field
            f_sm = np.heaviside(self.u_sm - self.theta_sm, 1)
            # conv_sm = self.dx * np.real(np.fft.ifft(np.fft.fft(f_sm) * self.w_hat_sm))
            conv_sm = self.dx * np.fft.ifftshift(np.real(np.fft.ifft(np.fft.fft(f_sm) * self.w_hat_sm)))
            self.h_u_sm += self.dt / self.tau_h_sm * f_sm
            self.u_sm += self.dt * (-self.u_sm + conv_sm + input1 + self.h_u_sm)
            
            # Signal that plot needs update
            self.plot_needs_update = True

        # Logging
        if int(self.time_counter) >= self.current_step:
            rospy.loginfo(f"Time {self.current_step:.1f}s | "
                         f"u_sm: [{self.u_sm.min():.2f}, {self.u_sm.max():.2f}] | "
                         f"u_d: [{self.u_d.min():.2f}, {self.u_d.max():.2f}]")
            self.current_step += 1

        # Check if finished
        if self.time_counter >= self.t_lim:
            if not self.is_finished:
                rospy.loginfo("Learning finished. Signaling main loop to save and exit.")
                self.is_finished = True
            # self.save_data()
            # rospy.signal_shutdown("Finished learning")

    def update_plot(self):
        """Update plot - must be called from main thread"""
        if not self.plot_needs_update:
            return
            
        try:
            with self.data_lock:
                # Copy data for plotting
                u_sm_copy = self.u_sm.copy()
                u_d_copy = self.u_d.copy()
                self.plot_needs_update = False
            
            # Update plot data
            self.line1.set_ydata(u_sm_copy)
            self.line2.set_ydata(u_d_copy)
            
            # Rescale if needed
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=False, scaley=True)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=True)
            
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
                self.save_data() # This is now called from the main thread!
                break # Exit the while loop to allow the node to terminate

            try:
                # if self.shutdown_requested:
                #     # If shutdown has started, don't try to plot anymore
                #     rate.sleep()
                #     continue
                # Update plot in main thread
                self.update_plot()
                
                # Process matplotlib events
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

        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(-self.x_lim, self.x_lim)
            ax.set_ylim(-2, 6)
            ax.set_xlabel("Objects", fontsize=10)
            ax.set_ylabel("u(x)", fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_xticks(object_all)
            ax.set_xticklabels(object_labels, rotation=45, ha='right', fontsize=9)
            for pos in object_all:
                ax.axvline(x=pos, color='gray', linestyle='--', alpha=0.3, linewidth=0.5)

        self.ax1.set_title("Sequence Memory Field", fontsize=12, fontweight='bold')
        self.ax2.set_title("Task Duration Field", fontsize=12, fontweight='bold')

    def save_data(self):
        self.shutdown_requested = True
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('fake_tiago_pkg')
            data_dir = os.path.join(pkg_path, "data_basic")
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            with self.data_lock:
                np.save(os.path.join(data_dir, f"u_sm_{timestamp}.npy"), self.u_sm)
                np.save(os.path.join(data_dir, f"u_d_{timestamp}.npy"), self.u_d)
            
            # Save final plot
            self.fig.savefig(os.path.join(data_dir, f"final_plot_{timestamp}.png"), 
                           dpi=150, bbox_inches='tight')
            
            rospy.loginfo(f"Saved data and plot in {data_dir}")
        except Exception as e:
            rospy.logerr(f"Error saving data: {e}")


if __name__ == "__main__":
    try:
        node = DNFLearningNode()
        # rospy.on_shutdown(node.save_data)
        rospy.loginfo("DNF Learning Node started. Waiting for inputs...")
        
        # Use custom spin instead of rospy.spin()
        node.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.close('all')