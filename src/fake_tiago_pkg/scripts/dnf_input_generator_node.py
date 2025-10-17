#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class DNFInputGenerator:
    """
    ROS node that generates time-varying input vectors for the DNF architecture.
    Produces three input matrices (two predefined, one dynamic) and publishes
    slices at each time step for downstream learning nodes.
    """

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dnf_input_generator_node', anonymous=True)

        # Publisher for combined input slices
        self.input_pub = rospy.Publisher(
            '/dnf_inputs', Float32MultiArray, queue_size=10
        )

        # Subscriber for threshold crossing events
        self.threshold_sub = rospy.Subscriber(
            '/threshold_crossings',
            Float32MultiArray,
            self.threshold_callback,
            queue_size=10
        )

        # Timer to publish slices periodically
        self.publish_rate = 1.0  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_slices)

        # Simulation parameters
        self.x_lim = 80
        self.t_lim = 15
        self.dx = 0.2
        self.dt = 0.1

        # Initialize Gaussian parameters for the predefined input matrices
        self.init_gaussian_params()

        # Spatial and temporal grids
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.t = np.arange(0, self.t_lim + self.dt, self.dt)

        # Generate input matrices
        self.input_matrix_1 = self.get_input_matrix(self.gaussian_params_1)
        self.input_matrix_2 = self.get_input_matrix(self.gaussian_params_2)
        self.input_matrix_3 = np.zeros((len(self.t), len(self.x)))  # dynamic updates

        self.current_time_index = 0
        rospy.loginfo("DNFInputGenerator initialized and ready to publish input slices")

    def init_gaussian_params(self):
        """Define Gaussian parameters for the first two matrices"""
        positions = [-40, 0, 40]
        amplitude = 5.0
        width = 2.0
        t_start_list_1 = [1, 4, 7]
        t_stop_list_1 = [2, 5, 8]
        t_start_list_2 = [1.5, 5, 7.5]
        t_stop_list_2 = [2.5, 6, 8.5]

        self.gaussian_params_1 = [
            {'center': pos, 'amplitude': amplitude, 'width': width,
             't_start': t_start, 't_stop': t_stop}
            for pos, t_start, t_stop in zip(positions, t_start_list_1, t_stop_list_1)
        ]

        self.gaussian_params_2 = [
            {'center': pos, 'amplitude': amplitude, 'width': width,
             't_start': t_start, 't_stop': t_stop}
            for pos, t_start, t_stop in zip(positions, t_start_list_2, t_stop_list_2)
        ]

        # Initially empty for dynamic updates
        self.gaussian_params_3 = []

    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        """Generate a Gaussian curve along the spatial dimension x"""
        return amplitude * np.exp(-((self.x - center) ** 2) / (2 * width ** 2))

    def get_input_matrix(self, params_list):
        """Generate an input matrix given a list of Gaussian parameters"""
        matrix = np.zeros((len(self.t), len(self.x)))
        for params in params_list:
            center = params['center']
            amplitude = params['amplitude']
            width = params['width']
            t_start = params['t_start']
            t_stop = params['t_stop']
            for i, t_val in enumerate(self.t):
                if t_start <= t_val <= t_stop:
                    matrix[i, :] += self.gaussian(center, amplitude, width)
        return matrix

    def threshold_callback(self, msg):
        """Update dynamic input matrix based on threshold crossing events"""
        if not msg.data:
            rospy.logwarn("Received empty threshold crossing message")
            return

        value = msg.data[0]
        rospy.loginfo(f"Threshold crossing received: {value:.2f}")

        # Map threshold to input positions
        if -45 <= value <= -35:
            input_position = -40
        elif -5 <= value <= 5:
            input_position = 0
        elif 35 <= value <= 45:
            input_position = 40
        else:
            rospy.logwarn("Threshold crossing value outside expected ranges")
            return

        # Define delay for dynamic input
        delay_steps = 5
        t_start = self.t[self.current_time_index] + delay_steps * self.dt
        t_stop = t_start + 10 * self.dt

        # Update Gaussian params for dynamic input (matrix 3)
        self.gaussian_params_3 = [
            {'center': input_position, 'amplitude': 5.0, 'width': 2.0,
             't_start': t_start, 't_stop': t_stop}
        ]

        # Regenerate dynamic input matrix
        self.input_matrix_3 = self.get_input_matrix(self.gaussian_params_3)
        rospy.loginfo(f"Updated dynamic input (matrix 3): {self.gaussian_params_3}")

    def publish_slices(self, event):
        """Publish combined slices of all three matrices"""
        if self.current_time_index >= len(self.t):
            rospy.loginfo("Completed publishing all input slices")
            self.timer.shutdown()
            return

        combined = [
            self.input_matrix_1[self.current_time_index].tolist(),
            self.input_matrix_2[self.current_time_index].tolist(),
            self.input_matrix_3[self.current_time_index].tolist()
        ]

        msg = Float32MultiArray()
        msg.data = [item for sublist in combined for item in sublist]

        self.input_pub.publish(msg)

        rospy.loginfo(
            f"[t={self.t[self.current_time_index]:.2f}] "
            f"Max1={self.input_matrix_1[self.current_time_index].max():.2f}, "
            f"Max2={self.input_matrix_2[self.current_time_index].max():.2f}, "
            f"Max3={self.input_matrix_3[self.current_time_index].max():.2f}"
        )

        self.current_time_index += 1

def main():
    try:
        node = DNFInputGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
