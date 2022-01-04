import matplotlib.pyplot as plt
import rospy
#import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
#from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'r')
        self.ln2, = plt.plot([], [], 'b:')
        self.x_data, self.y1_data , self.y2_data = [] , [], []

    def plot_init(self):
        self.ax.set_xlabel('Sequence')
        self.ax.set_ylabel('R: filtered, dotted B: Raw')
        self.ax.set_xlim(0, 1000)
#        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_ylim(-3.15, 3.15)
        return self.ln, self.ln2

    def odom_callback(self, msg):
#        self.y1_data.append(msg.pose.pose.position.x)
#        self.y1_data.append(msg.pose.pose.position.y)
        self.y1_data.append(msg.pose.pose.orientation.z)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)

    def no_callback(self, msg):
#        self.y2_data.append(msg.pose.pose.position.x)
#        self.y2_data.append(msg.pose.pose.position.y)
        self.y2_data.append(msg.pose.pose.orientation.z)

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y1_data)
        self.ln2.set_data(self.x_data, self.y2_data)
        return self.ln, self.ln2


rospy.init_node('imu_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/odometry/filtered', Odometry, vis.odom_callback)
sub = rospy.Subscriber('/odom', Odometry, vis.no_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 
