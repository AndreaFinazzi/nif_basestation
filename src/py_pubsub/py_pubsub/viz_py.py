from textwrap import dedent
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nif_msgs.msg import DynamicTrajectory
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from nif_msgs.msg import Telemetry
from nif_msgs.msg import SystemStatus
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray

import matplotlib.pyplot as plt
#import tf
from tf_transformations import quaternion_matrix
import tf_transformations
import numpy as np
from matplotlib.animation import FuncAnimation
import csv
import math

IMS = 0
LOR = 1
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "IMS_SIM" or track_id == "ims_sim":
    track = IMS_SIM
elif track_id == "LVMS" or track_id == "lvms":
    track = LVMS
elif track_id == "LVMS_SIM" or track_id == "lvms_sim":
    track = LVMS_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))

csv_trackboundaries_inner = ''
csv_trackboundaries_outer = ''
csv_trackboundaries_pit_inner = ''
csv_trackboundaries_pit_outer = ''

track_x_bias = 0
track_y_bias = 0

ax_x_lim = (0, 0)
ax_y_lim = (0, 0)

if track == LOR:
    csv_trackboundaries_inner = '/workspace/src/py_pubsub/py_pubsub/ims_inner_211004_cy_modified_wpt.csv'
    csv_trackboundaries_outer = '/workspace/src/py_pubsub/py_pubsub/ims_outer_211004_cy_modified_wpt.csv'
    csv_trackboundaries_pit_inner = '/workspace/src/py_pubsub/py_pubsub/ims_pit_inner_wpt.csv'
    csv_trackboundaries_pit_outer = '/workspace/src/py_pubsub/py_pubsub/ims_pit_outer_wpt.csv'

elif track == IMS:
    csv_trackboundaries_inner = '/workspace/src/py_pubsub/py_pubsub/ims_inner_211004_cy_modified_wpt.csv'
    csv_trackboundaries_outer = '/workspace/src/py_pubsub/py_pubsub/ims_outer_211004_cy_modified_wpt.csv'
    csv_trackboundaries_pit_inner = '/workspace/src/py_pubsub/py_pubsub/ims_pit_inner_wpt.csv'
    csv_trackboundaries_pit_outer = '/workspace/src/py_pubsub/py_pubsub/ims_pit_outer_wpt.csv'
    track_x_bias = 1600  # IMS
    track_y_bias = 320  # IMS

    ax_x_lim = (-850, 850)
    ax_y_lim = (-850, 850)

elif track == IMS_SIM:
    config_file = 'config_lgsim.yaml'

elif track == LVMS:
    csv_trackboundaries_inner = '/workspace/src/py_pubsub/assets/LVMS/trackboundary_left.csv'
    csv_trackboundaries_outer = '/workspace/src/py_pubsub/assets/LVMS/trackboundary_right.csv'
    csv_trackboundaries_pit_inner = '/workspace/src/py_pubsub/assets/LVMS/pit_trackboundary_left.csv'
    csv_trackboundaries_pit_outer = '/workspace/src/py_pubsub/assets/LVMS/pit_trackboundary_right.csv'
    track_x_bias = 0  # IMS
    track_y_bias = 0  # IMS

    ax_x_lim = (0, 1000)
    ax_y_lim = (0, -1000)

elif track == LVMS_SIM:
    config_file = 'config_lvms_sim.yaml'

else:
    raise RuntimeError("ERROR: invalid track provided: {}".format(track))


class Visualiser(Node):
    def __init__(self):
        self.range = np.pi / 5
        self.x_pred = 0
        self.y_pred = 0
        self.desired_speed = 0
        self.incre = 0.05
        self.steering = 0
        self.radius = 0
        self.wheelbase = 3.0
        self.max = 10**10
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2
        self.x_data, self.y_data = [], []
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.x_bias = track_x_bias
        self.y_bias = track_y_bias
        # self.x_bias = 0.0 # LOR
        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw))

        self.ln1i, = self.ax1.plot([], [], 'g--')
        self.ln1o, = self.ax1.plot([], [], 'g--')
        self.ln1y, = self.ax1.plot([], [], color='0.5')
        self.ln1t, = self.ax1.plot([], [], color='0.5')

        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw))
        self.ln2i, = self.ax2.plot([], [], 'r-')
        self.ln2o, = self.ax2.plot([], [], 'g-')
        self.ln2p, = self.ax2.plot([], [], 'b-')
        self.ln2y, = self.ax2.plot([], [], 'k-')
        self.ln2t, = self.ax2.plot([], [], 'k-')
        self.ln2u, = self.ax2.plot([], [], 'c-')
        self.ln2oppo, = self.ax2.plot([], [], 'm-.')
        self.ln2m, = self.ax2.plot([], [], 'r*')  # cyan
        self.ln2eo, = self.ax2.plot([], [], 'c-')
        self.ln2eop, = self.ax2.plot([], [], 'c-')

        self.x_data, self.y_data = [], []
        self.x_in_data, self.y_in_data = [], []
        self.x_out_data, self.y_out_data = [], []
        self.x_pred_data, self.y_pred_data = [], []
        self.x_pit_in_data, self.y_pit_in_data = [], []
        self.x_pit_out_data, self.y_pit_out_data = [], []
        self.x_path_data, self.y_path_data = [], []
        self.x_oppo_path_data, self.y_oppo_path_data = [], []
        self.obs_x, self.obs_y = [], []
        self.plotEO_x, self.plotEO_y = [], []
        self.plotEOP_x, self.plotEOP_y = [], []
        self.cnt = 0
        self.out_cout = 0
        self.CO_idx = 0
        self.CO_idx_n = 0
        self.min_dist_O = 99999
        self.error_O = 99999
        self.wall_dist_sensing = 0
        self.gps_code_color = 'black'
        self.lateral_error = 0
        self.mission_code = 0

        # for dynamic trajectory (ego vehicle's planned result)
        self.traj_x, self.traj_y = [], []
        self.lat_planning_type = -1
        self.long_planning_type = -1
        self.planning_target_path_id = -1

        f = open(csv_trackboundaries_inner, 'r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt != 0):
                self.x_in_data.append(float(line[0]) + self.x_bias)
                self.y_in_data.append(float(line[1]) + self.y_bias)
            self.cnt = 1
        f.close()
        self.cnt = 0
        f = open(csv_trackboundaries_outer, 'r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt != 0):
                self.x_out_data.append(float(line[0]) + self.x_bias)
                self.y_out_data.append(float(line[1]) + self.y_bias)
                self.out_cout += 1
            self.cnt = 1
        f.close()
        self.cnt = 0
        f = open(csv_trackboundaries_pit_inner, 'r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt != 0):
                self.x_pit_in_data.append(float(line[0]) + self.x_bias)
                self.y_pit_in_data.append(float(line[1]) + self.y_bias)
            self.cnt = 1
        f.close()
        self.cnt = 0
        f = open(csv_trackboundaries_pit_outer, 'r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt != 0):
                self.x_pit_out_data.append(float(line[0]) + self.x_bias)
                self.y_pit_out_data.append(float(line[1]) + self.y_bias)
            self.cnt = 1
        f.close()
        self.cnt = 0

    def plot_init1b(self):
        self.ax1.set_xlim(ax_x_lim)
        self.ax1.set_ylim(ax_y_lim)
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        # self.ax1.invert_xaxis()
        return self.ln1b

    def plot_init1i(self):
        self.ax1.set_xlim(ax_x_lim)
        self.ax1.set_ylim(ax_y_lim)
        # self.ax1.set_xlim(-250,250)
        # self.ax1.set_ylim(-250,50)
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        # self.ax1.invert_xaxis()
        return self.ln1i

    def plot_init1o(self):
        self.ax1.set_xlim(ax_x_lim)
        self.ax1.set_ylim(ax_y_lim)
        # self.ax1.set_xlim(-250,250)
        # self.ax1.set_ylim(-250,50)
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        # self.ax1.invert_xaxis()
        return self.ln1o

    def plot_init1y(self):
        self.ax1.set_xlim(ax_x_lim)
        self.ax1.set_ylim(ax_y_lim)
        # self.ax1.set_xlim(-250,250)
        # self.ax1.set_ylim(-250,50)
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        # self.ax1.invert_xaxis()
        return self.ln1y

    def plot_init1t(self):
        self.ax1.set_xlim(ax_x_lim)
        self.ax1.set_ylim(ax_y_lim)
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        # self.ax1.invert_xaxis()
        return self.ln1t

    def plot_init2b(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2b

    def plot_init2i(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2i

    def plot_init2o(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2o

    def plot_init2p(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2p

    def plot_init2y(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2y

    def plot_init2t(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2t

    def plot_init2u(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2u

    def plot_init2oppo(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2oppo

    def plot_init2m(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        # self.ax2.invert_xaxis()
        return self.ln2m

    def plot_init2eo(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        return self.ln2eo

    def plot_init2eop(self):
        self.ax2.set_xlim(ax_x_lim)
        self.ax2.set_ylim(ax_y_lim)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        return self.ln2eop

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw

    def path_callback(self, msg):
        # print("path_callback")
        self.x_path_data = []
        self.y_path_data = []
        for i in range(np.size(msg.poses)):
            self.x_path_data.append(msg.poses[i].pose.position.x + self.x_bias)
            self.y_path_data.append(msg.poses[i].pose.position.y + self.y_bias)

    def traj_callback(self, msg):
        # print("path_callback")
        self.traj_x = []
        self.traj_y = []

        for i in range(np.size(msg.trajectory_path.poses)):
            self.traj_x.append(msg.trajectory_path.poses[i].pose.position.x + self.x_bias)
            self.traj_y.append(msg.trajectory_path.poses[i].pose.position.y + self.y_bias)

        self.lat_planning_type = msg.lat_planning_type
        self.long_planning_type = msg.longi_planning_type
        self.planning_target_path_id = msg.planning_target_path_type

    def oppo_prediction_path_callback(self, msg):
        # print("path_callback")
        self.x_oppo_path_data = []
        self.y_oppo_path_data = []
        for i in range(np.size(msg.poses)):
            self.x_oppo_path_data.append(msg.poses[i].pose.position.x + self.x_bias)
            self.y_oppo_path_data.append(msg.poses[i].pose.position.y + self.y_bias)

    def tele_callback(self, msg):
        # print("tele_callback")
        self.x_pred_data = []
        self.y_pred_data = []
        self.steering = msg.kinematic.steering_wheel_angle_deg / 9.5 * 3.141592 / 180
        self.lateral_error = round(msg.control.crosstrack_error, 3)
        if (self.steering == 0):
            self.radius = self.max
        else:
            self.radius = self.wheelbase / np.tan(self.steering)

        for i in np.arange(0, self.range, self.incre):
            if (self.radius == self.max):
                self.x_pred = i * 20
                self.y_pred = 0
                self.x_pred_data.append(self.x + (self.x_pred * np.cos(self.yaw)) - (self.y_pred * np.sin(self.yaw)))
                self.y_pred_data.append(self.y + (self.x_pred * np.sin(self.yaw)) + (self.y_pred * np.cos(self.yaw)))
            else:
                if (self.radius > 0):
                    self.x_pred = self.radius * np.sin(i)
                    self.y_pred = -self.radius * np.cos(i) + self.radius
                    self.x_pred_data.append(self.x + (self.x_pred * np.cos(self.yaw)) - (self.y_pred * np.sin(self.yaw)))
                    self.y_pred_data.append(self.y + (self.x_pred * np.sin(self.yaw)) + (self.y_pred * np.cos(self.yaw)))
                else:
                    self.x_pred = -self.radius * np.sin(i)
                    self.y_pred = -self.radius * np.cos(i) + self.radius
                    self.x_pred_data.append(self.x + (self.x_pred * np.cos(self.yaw)) - (self.y_pred * np.sin(self.yaw)))
                    self.y_pred_data.append(self.y + (self.x_pred * np.sin(self.yaw)) + (self.y_pred * np.cos(self.yaw)))

        # POSITION
        self.yaw = self.getYaw(msg.localization.odometry.pose)
        self.x = msg.localization.odometry.pose.position.x + self.x_bias
        self.y = msg.localization.odometry.pose.position.y + self.y_bias
        self.x_data, self.y_data = [], []
        self.y_data.append(self.y)
        self.x_data.append(self.x)

        self.plotEO_x, self.plotEO_y = [], []
        min_dist_O = 99999

        for idx in range(self.out_cout):
            dist = ((self.x - self.x_out_data[idx])**2 + (self.y - self.y_out_data[idx])**2)**0.5
            if dist < min_dist_O:
                min_dist_O = dist
                self.CO_idx = idx
        if self.CO_idx != (self.out_cout - 1):
            self.CO_idx_n = self.CO_idx + 1
        else:
            self.CO_idx_n = 0

        m = (self.y_out_data[self.CO_idx_n] - self.y_out_data[self.CO_idx]) / (self.x_out_data[self.CO_idx_n] - self.x_out_data[self.CO_idx])
        d = (-1) * m * self.x_out_data[self.CO_idx] + self.y_out_data[self.CO_idx]
        self.error_O = abs(m * self.x - self.y + d) / (m**2 + 1)**0.5
        self.plotEO_x.append(self.x_out_data[self.CO_idx])
        self.plotEO_x.append(self.x_out_data[self.CO_idx_n])
        self.plotEO_y.append(self.y_out_data[self.CO_idx])
        self.plotEO_y.append(self.y_out_data[self.CO_idx_n])
        m2 = (-1) / m
        d2 = (-1) * m2 * self.x + self.y
        self.px = (d - d2) / (m2 - m)
        self.py = m * self.px + d
        self.plotEOP_x, self.plotEOP_y = [], []
        self.plotEOP_x.append(self.x)
        self.plotEOP_x.append(self.px)
        self.plotEOP_y.append(self.y)
        self.plotEOP_y.append(self.py)

        # SPEED_Desired
        self.desired_speed = round(msg.control.desired_velocity_mps, 3)

        # SPEED
        self.speed = round(msg.kinematic.wheel_speed_mps, 3)

        # GPS
        self.gps_uncertainty = round(msg.localization.uncertainty, 3)

        self.gps_code = msg.localization.localization_status_code
        if self.gps_code < 80:
            self.gps_code_color = 'green'
        elif self.gps_code < 200:
            self.gps_code_color = 'orange'
        else:
            self.gps_code_color = 'red'

        # WALL DIST
        self.wall_dist_sensing = msg.localization.detected_outer_distance

        # FLUSH
        fig.canvas.flush_events()

    def system_callback(self, msg):
        self.mission_code = msg.mission_status.mission_status_code

    def marker_callback(self, msg):
        self.obs_x = []
        self.obs_y = []
        for i in range(np.size(msg.markers)):
            self.obs_x.append(self.x + (msg.markers[i].pose.position.x * np.cos(self.yaw)) - (msg.markers[i].pose.position.y * np.sin(self.yaw)))
            self.obs_y.append(self.y + (msg.markers[i].pose.position.x * np.sin(self.yaw)) + (msg.markers[i].pose.position.y * np.cos(self.yaw)))

    def update_plot1b(self, frame):
        self.ln1b.remove()

        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw * 180 / 3.1415 - 90), markersize=20, color='blue')

        self.ln1b.set_data(self.x_data, self.y_data)
        return self.ln1b

    def update_plot1i(self, frame):
        self.ln1i.set_data(self.x_in_data, self.y_in_data)
        return self.ln1i

    def update_plot1o(self, frame):
        self.ln1o.set_data(self.x_out_data, self.y_out_data)
        return self.ln1o

    def update_plot1y(self, frame):
        self.ln1y.set_data(self.x_pit_in_data, self.y_pit_in_data)
        return self.ln1y

    def update_plot1t(self, frame):
        self.ln1t.set_data(self.x_pit_out_data, self.y_pit_out_data)
        return self.ln1t

    def update_plot2b(self, frame):
        self.ln2b.remove()
        self.ax2.clear()
        self.ax2.axis('off')
        self.ax2.axis('scaled')
        self.ln2i, = self.ax2.plot([], [], 'r-')
        self.ln2o, = self.ax2.plot([], [], 'g-')
        self.ln2p, = self.ax2.plot([], [], 'b-')
        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw * 180 / 3.1415 - 90), markersize=12, color='blue')
        self.ln2y, = self.ax2.plot([], [], 'k-')
        self.ln2t, = self.ax2.plot([], [], 'k-')

        # self.ln2u, = self.ax2.plot([], [], 'c-')

        self.ln2oppo, = self.ax2.plot([], [], 'm-.')
        self.ln2m, = self.ax2.plot([], [], 'rs', markersize=10)
        self.ln2eo, = self.ax2.plot([], [], marker=(100, 0, self.yaw * 180 / 3.1415 - 90), markersize=8, color='purple', linewidth=2)
        self.ln2eop, = self.ax2.plot([], [], 'c-', linewidth=3)
        self.ax2.axis([self.x - 50, self.x + 50, self.y - 50, self.y + 50])
        # self.ax2.invert_xaxis()
        self.ln2b.set_data(self.x_data, self.y_data)
        # colors
        fig_width, fig_height = plt.gcf().get_size_inches()
        if self.gps_code_color == 'red':
            self.fig.patch.set_facecolor('coral')
        elif self.gps_code_color == 'orange':
            self.fig.patch.set_facecolor('yellow')
        else:
            self.fig.patch.set_facecolor('white')
        GPS_status = 'EMPTY'
        if self.gps_code == 71:
            GPS_status = 'BEST_STATUS'
        elif self.gps_code == 72:
            GPS_status = 'SENSOR_FUSION'
        elif self.gps_code == 73:
            GPS_status = 'ONLY_TOP'
        elif self.gps_code == 74:
            GPS_status = 'ONLY_BOTTOM'
        elif self.gps_code == 150:
            GPS_status = 'GPS_HIGH_ERROR'
        elif self.gps_code == 225:
            GPS_status = 'UNCERTAINTY_TOO_HIGH'
        elif self.gps_code == 235:
            GPS_status = 'NO_CONVERGED'
        elif self.gps_code == 255:
            GPS_status = 'NO_SENSOR_INITIALIZED'
        else:
            GPS_status = 'UNKNOWN'

        # Trajctory planning info
        traj_lat_status = "UNKNOWN"
        traj_long_status = "UNKNOWN"
        traj_target_path_id_status = "UNKNOWN"

        if self.lat_planning_type == DynamicTrajectory.LATERAL_PLANNING_TYPE_KEEP:
            traj_lat_status = "Lat: Keep path"
            self.ln2u, = self.ax2.plot([], [], 'c-')

        elif self.lat_planning_type == DynamicTrajectory.LATERAL_PLANNING_TYPE_MERGE:
            traj_lat_status = "Lat: Merge back"
            self.ln2u, = self.ax2.plot([], [], color='c', linestyle=(0, (5, 10)))

        elif self.lat_planning_type == DynamicTrajectory.LATERAL_PLANNING_TYPE_CHANGE_PATH:
            traj_lat_status = "Lat: Change path"
            self.ln2u, = self.ax2.plot([], [], color='c', linestyle=(0, (5, 10)))

        else:
            traj_lat_status = "Lat: Unknown"
            self.ln2u, = self.ax2.plot([], [], 'k-')

        if self.long_planning_type == DynamicTrajectory.LONGITUDINAL_PLANNING_TYPE_STRAIGHT:
            traj_long_status = " Long: Drive"
        elif self.long_planning_type == DynamicTrajectory.LONGITUDINAL_PLANNING_TYPE_FOLLOW:
            traj_long_status = " Long: Follow"
        elif self.long_planning_type == DynamicTrajectory.LONGITUDINAL_PLANNING_TYPE_ESTOP:
            traj_long_status = " Long: Estop"
        else:
            traj_long_status = " Long: Unknown"

        if self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_RACELINE:
            traj_target_path_id_status = " Target Path: RaceLine"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_CENTER:
            traj_target_path_id_status = " Target Path: Center"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_RIGHT:
            traj_target_path_id_status = " Target Path: Right-side"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_LEFT:
            traj_target_path_id_status = " Target Path: Left-side"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_CENTER_RIGHT:
            traj_target_path_id_status = " Target Path: Right-center-side"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_CENTER_LEFT:
            traj_target_path_id_status = " Target Path: Left-center-side"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_RACE_READY:
            traj_target_path_id_status = " Target Path: Race ready"
        elif self.planning_target_path_id == DynamicTrajectory.PLANNING_TARGET_PATH_DEFENDER:
            traj_target_path_id_status = " Target Path: Defender line"
        else:
            traj_target_path_id_status = " Target Path: Unknown"

        self.ax2.text(-0.0, 1.1, r'Nav: {} ({})'.format(GPS_status, self.gps_code), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color=self.gps_code_color)  # fig_width*0.15 , 0.9 0.85 ,0.8
        self.ax2.text(-0.0, 1.05, r'Nav uncertainty:{}'.format(self.gps_uncertainty, 3), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(-0.0, 1, r'Speed:{}[mps] / {}[mph]'.format(self.speed, round(self.speed * 2.23694, 2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(-0.0, 0.95, r'Desired:{}[mps] / {}[mph]'.format(self.desired_speed, round(self.desired_speed * 2.23694, 2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(0.6, 1.1, r'Cross Track Error   [m]:{}'.format(self.lateral_error), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(0.6, 1.05, r'Outlane Dist Nav   [m]:{}'.format(round(self.error_O, 2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        # self.ax2.text(0.6, 1.05, r'Outlane lateral Dist [m]:{}'.format(round(self.error_O,2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        if self.wall_dist_sensing < 0.01:
            self.ax2.text(0.6, 0.95, r'Outlane Dist LiDAR[m]: None', transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        else:
            self.ax2.text(0.6, 0.95, r'Outlane Dist LiDAR[m]:{}'.format(round(self.wall_dist_sensing, 2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')

        # if self.wall_dist_sensing < 0.01:
        #     self.ax2.text(0.6, 1.0, r'Outlane Dist Error  [m]: None', transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        # else:
        #     self.ax2.text(0.6, 1.0, r'Outlane Dist Error  [m]:{}'.format(round(np.abs(self.wall_dist_sensing - self.error_O), 2)), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')

        # Planning type
        self.ax2.text(0.6, 1.0, r'Planning Info : ', transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(0.6, 0.95, r'{} / {} / {}'.format(traj_lat_status, traj_long_status, traj_target_path_id_status), transform=self.ax2.transAxes, horizontalalignment='left', size=12, color='black')

        legend = self.ax1.legend([self.ln2i, self.ln2o, self.ln2y, self.ln2p, self.ln2u, self.ln2oppo, self.ln2m], ['Inner lane', 'Outer lane', 'Pit lane', 'Pediction path', 'Waypoint', 'Prediction Path', 'Obstacle'], bbox_to_anchor=(1.0, 0.8), ncol=2, fontsize=10, frameon=False)
        self.fig.patch.set_alpha(0.5)

        Mission_status = 'EMPTY'
        if self.mission_code == 0:
            Mission_status = 'MISSION_RACE'
        elif self.mission_code == 50:
            Mission_status = 'MISSION_STANDBY'
        elif self.mission_code == 60:
            Mission_status = 'MISSION_PIT_IN'
        elif self.mission_code == 65:
            Mission_status = 'MISSION_PIT_STANDBY'
        elif self.mission_code == 70:
            Mission_status = 'MISSION_PIT_OUT'
        elif self.mission_code == 75:
            Mission_status = 'MISSION_PIT_TO_TRACK'
        elif self.mission_code == 128:
            Mission_status = 'MISSION_SLOW_DRIVE'
        elif self.mission_code == 200:
            Mission_status = 'MISSION_COMMANDED_STOP'
        elif self.mission_code == 250:
            Mission_status = 'MISSION_EMERGENCY_STOP'
        elif self.mission_code == 400:
            Mission_status = 'MISSION_COLLISION_AVOIDNACE'
        elif self.mission_code == 500:
            Mission_status = 'MISSION_TIRE_WARMUP'
        elif self.mission_code == 777:
            Mission_status = 'MISSION_TEST'
        elif self.mission_code == 50000:
            Mission_status = 'MISSION_INIT'
        elif self.mission_code == 65000:
            Mission_status = 'MISSION_PIT_INIT'
        elif self.mission_code == 65535:
            Mission_status = 'MISSION_DEFAULT'
        else:
            Mission_status = 'UNKNOWN'

        self.ax2.text(0, 1.23, r'Mission Status:', transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')
        self.ax2.text(0, 1.2, r'{}'.format(Mission_status), transform=self.ax2.transAxes, horizontalalignment='left', size=18, color='black')

        return self.ln2b

    def update_plot2i(self, frame):
        self.ln2i.set_data(self.x_in_data, self.y_in_data)
        return self.ln2i

    def update_plot2o(self, frame):
        self.ln2o.set_data(self.x_out_data, self.y_out_data)
        return self.ln2o

    def update_plot2p(self, frame):
        self.ln2p.set_data(self.x_pred_data, self.y_pred_data)
        return self.ln2p

    def update_plot2y(self, frame):
        self.ln2y.set_data(self.x_pit_in_data, self.y_pit_in_data)
        return self.ln2y

    def update_plot2t(self, frame):
        self.ln2t.set_data(self.x_pit_out_data, self.y_pit_out_data)
        return self.ln2t
    # def update_plot2u(self, frame):
    #     self.ln2u.set_data(self.x_path_data, self.y_path_data)
    #     return self.ln2u

    def update_plot2traj(self, frame):
        self.ln2u.set_data(self.traj_x, self.traj_y)
        return self.ln2u

    def update_plot2oppo(self, frame):
        self.ln2oppo.set_data(self.x_oppo_path_data, self.y_oppo_path_data)
        return self.ln2oppo

    def update_plot2m(self, frame):
        self.ln2m.set_data(self.obs_x, self.obs_y)
        return self.ln2m

    def update_plot2eo(self, frame):
        self.ln2eo.set_data(self.plotEO_x, self.plotEO_y)
        return self.ln2eo

    def update_plot2eop(self, frame):
        self.ln2eop.set_data(self.plotEOP_x, self.plotEOP_y)
        return self.ln2eop


fig, (ax1, ax2) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [1, 3]})

vis = Visualiser()
ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b, interval=50)
ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i, interval=50)
ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o, interval=50)
ani1y = FuncAnimation(vis.fig, vis.update_plot1y, init_func=vis.plot_init1y, interval=50)
ani1t = FuncAnimation(vis.fig, vis.update_plot1t, init_func=vis.plot_init1t, interval=50)
ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b, interval=50)
ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i, interval=50)
ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o, interval=50)
ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p, interval=50)
ani2y = FuncAnimation(vis.fig, vis.update_plot2y, init_func=vis.plot_init2y, interval=50)
ani2t = FuncAnimation(vis.fig, vis.update_plot2t, init_func=vis.plot_init2t, interval=50)
ani2u = FuncAnimation(vis.fig, vis.update_plot2u, init_func=vis.plot_init2u, interval=50)
ani2oppo = FuncAnimation(vis.fig, vis.update_plot2oppo, init_func=vis.plot_init2oppo, interval=50)
ani2m = FuncAnimation(vis.fig, vis.update_plot2m, init_func=vis.plot_init2m, interval=50)
ani2eo = FuncAnimation(vis.fig, vis.update_plot2eo, init_func=vis.plot_init2eo, interval=50)

vis.ax1.axis('scaled')
vis.ax2.axis('scaled')
vis.ax1.axis('off')
vis.ax2.axis('off')
plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
fig.show()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('Visualiser')
    sub_odom = node.create_subscription(Telemetry, '/nif_telemetry/telemetry', vis.tele_callback, qos_profile_sensor_data)
    sub_path = node.create_subscription(Path, '/nif_telemetry/path_global', vis.path_callback, qos_profile_sensor_data)
    sub_traj = node.create_subscription(DynamicTrajectory, '/nif_telemetry/traj_global', vis.traj_callback, qos_profile_sensor_data)
    sub_oppo_prediction = node.create_subscription(Path, '/nif_telemetry/oppo_prediction', vis.oppo_prediction_path_callback, qos_profile_sensor_data)
    sub_marker = node.create_subscription(MarkerArray, '/nif_telemetry/perception_result', vis.marker_callback, qos_profile_sensor_data)
    sub_system = node.create_subscription(SystemStatus, '/nif_telemetry/system_status', vis.system_callback, qos_profile_sensor_data)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
