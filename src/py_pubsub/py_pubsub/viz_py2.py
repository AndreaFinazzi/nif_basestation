import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from nif_msgs.msg import Telemetry
from rclpy.qos import qos_profile_sensor_data

import matplotlib.pyplot as plt
#import tf
from tf_transformations import quaternion_matrix
import tf_transformations
import numpy as np
from matplotlib.animation import FuncAnimation
import csv


class Visualiser(Node):
    def __init__(self):      
        self.range =np.pi
        self.x_pred=0
        self.y_pred=0
        self.incre=0.05
        self.steering=0
        self.radius=0
        self.wheelbase = 3.0
        self.max = 10**10
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2 
        self.x_data, self.y_data = [] , []
        self.yaw=0
        self.x=0
        self.y=0
        # self.x_bias = 1600 #IMS
        self.x_bias = 0.0 # LOR
        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw))
        self.ln1i, = self.ax1.plot([], [], 'r-')
        self.ln1o, = self.ax1.plot([], [], 'g-')
        self.ln1y, = self.ax1.plot([], [], 'k-')
        self.ln1t, = self.ax1.plot([], [], 'k-')
                        
        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw))
        self.ln2i, = self.ax2.plot([], [], 'r-')
        self.ln2o, = self.ax2.plot([], [], 'g-')
        self.ln2p, = self.ax2.plot([], [], 'b-')
        self.ln2y, = self.ax2.plot([], [], 'k-')
        self.ln2t, = self.ax2.plot([], [], 'k-')
        self.ln2u, = self.ax2.plot([], [], 'm-')

        self.x_data, self.y_data = [] , []
        self.x_in_data, self.y_in_data = [] , []
        self.x_out_data, self.y_out_data = [] , []
        self.x_pred_data, self.y_pred_data = [] , []
        self.x_pit_in_data, self.y_pit_in_data = [] , []
        self.x_pit_out_data, self.y_pit_out_data = [] , []
        self.x_path_data, self.y_path_data = [] , []                                 
        self.cnt=0
        self.gps_code_color = 'black'

        # f = open('/workspace/src/py_pubsub/py_pubsub/ims_inner_211004_cy_modified_wpt.csv','r')
        f = open('/workspace/src/py_pubsub/py_pubsub/lor_inner_dense_211005_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_in_data.append(float(line[0])+self.x_bias)
                self.y_in_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0
        # f = open('/workspace/src/py_pubsub/py_pubsub/ims_outer_211004_cy_modified_wpt.csv','r')
        f = open('/workspace/src/py_pubsub/py_pubsub/lor_outer_dense_211005_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_out_data.append(float(line[0])+self.x_bias)
                self.y_out_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0
        f = open('/workspace/src/py_pubsub/py_pubsub/lor_pit_inner_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_pit_in_data.append(float(line[0])+self.x_bias)
                self.y_pit_in_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0
        f = open('/workspace/src/py_pubsub/py_pubsub/lor_pit_outer_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_pit_out_data.append(float(line[0])+self.x_bias)
                self.y_pit_out_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0        
    def plot_init1b(self):
       
        # self.ax1.set_xlim(-850,850)
        # self.ax1.set_ylim(-850,850)
        self.ax1.set_xlim(-500,500)
        self.ax1.set_ylim(-500,100)        
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        #self.ax1.invert_xaxis()
        return self.ln1b

    def plot_init1i(self):
        # self.ax1.set_xlim(-850,850)
        # self.ax1.set_ylim(-850,850)
        self.ax1.set_xlim(-250,250)
        self.ax1.set_ylim(-250,50)                
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        #self.ax1.invert_xaxis()
        return self.ln1i

    def plot_init1o(self):
        # self.ax1.set_xlim(-850,850)
        # self.ax1.set_ylim(-850,850)
        self.ax1.set_xlim(-250,250)
        self.ax1.set_ylim(-250,50)                
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        #self.ax1.invert_xaxis()
        return self.ln1o
    def plot_init1y(self):
        # self.ax1.set_xlim(-850,850)
        # self.ax1.set_ylim(-850,850)
        self.ax1.set_xlim(-250,250)
        self.ax1.set_ylim(-250,50)                
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        #self.ax1.invert_xaxis()
        return self.ln1y
    def plot_init1t(self):
        # self.ax1.set_xlim(-850,850)
        # self.ax1.set_ylim(-850,850)
        self.ax1.set_xlim(-250,250)
        self.ax1.set_ylim(-250,50)                
        self.ax1.spines['left'].set_position('zero')
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        self.ax1.spines['top'].set_color('none')
        #self.ax1.invert_xaxis()
        return self.ln1t               

    def plot_init2b(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2b    

    def plot_init2i(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2i    

    def plot_init2o(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2o    

    def plot_init2p(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2p
    def plot_init2y(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2y   
    def plot_init2t(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2t                       
    def plot_init2u(self):
        self.ax2.set_xlim(-300,300)
        self.ax2.set_ylim(-300,300)
        self.ax2.spines['left'].set_position('zero')
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        self.ax2.spines['top'].set_color('none')
        #self.ax2.invert_xaxis()
        return self.ln2u

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw

    # def steering_callback(self, msg):
    #     self.x_pred_data=[]
    #     self.y_pred_data=[]
    #     self.steering = msg.data
    #     if (self.steering==0):
    #         self.radius = self.max
    #     else:
    #         self.radius = self.wheelbase / np.tan(self.steering)
       
    #     for i in np.arange(0,self.range,self.incre):
    #         if (self.radius==self.max):
    #             self.x_pred=i*20
    #             self.y_pred=0
    #             self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
    #             self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))
    #         else:
    #             if (self.radius>0):
    #                 self.x_pred=self.radius*np.sin(i)
    #                 self.y_pred=-self.radius*np.cos(i)+self.radius
    #                 self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
    #                 self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))
    #             else:
    #                 self.x_pred=-self.radius*np.sin(i)
    #                 self.y_pred=-self.radius*np.cos(i)+self.radius
    #                 self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
    #                 self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))
                    
    def path_callback(self, msg):
        print("path_callback")
        self.x_path_data=[]
        self.y_path_data=[]
        for i in range(np.size(msg.poses)):
            self.x_path_data.append(msg.poses[i].pose.position.x)
            self.y_path_data.append(msg.poses[i].pose.position.y)

    def tele_callback(self, msg):
        print("tele_callback")
        self.x_pred_data=[]
        self.y_pred_data=[]
        self.steering = msg.kinematic.steering_wheel_angle_deg/9.5 * 3.141592/180
        if (self.steering==0):
            self.radius = self.max
        else:
            self.radius = self.wheelbase / np.tan(self.steering)
       
        for i in np.arange(0,self.range,self.incre):
            if (self.radius==self.max):
                self.x_pred=i*20
                self.y_pred=0
                self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
                self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))
            else:
                if (self.radius>0):
                    self.x_pred=self.radius*np.sin(i)
                    self.y_pred=-self.radius*np.cos(i)+self.radius
                    self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
                    self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))
                else:
                    self.x_pred=-self.radius*np.sin(i)
                    self.y_pred=-self.radius*np.cos(i)+self.radius
                    self.x_pred_data.append(self.x+(self.x_pred*np.cos(self.yaw))-(self.y_pred*np.sin(self.yaw)))
                    self.y_pred_data.append(self.y+(self.x_pred*np.sin(self.yaw))+(self.y_pred*np.cos(self.yaw)))

        #POSITION
        self.yaw = self.getYaw(msg.localization.odometry.pose)
        self.x=msg.localization.odometry.pose.position.x
        self.y=msg.localization.odometry.pose.position.y # +320 (IMS)
        self.x_data, self.y_data = [] , []
        self.y_data.append(self.y)
        self.x_data.append(self.x)

        #SPEED
        self.speed = round(msg.kinematic.wheel_speed_mps,3)

        #GPS
        self.gps_uncertainty = round(msg.localization.uncertainty,3)

        self.gps_code = msg.localization.localization_status_code
        if self.gps_code < 80 :
            self.gps_code_color = 'green'
        elif  self.gps_code < 200 :
            self.gps_code_color = 'orange'
        else:
            self.gps_code_color = 'red'

        #FLUSH
        fig.canvas.flush_events()
    	
    # def odom_callback(self, msg):
    #     print("Callback1")
    #     self.yaw = self.getYaw(msg.pose.pose)
    #     self.x=msg.pose.pose.position.x
    #     self.y=msg.pose.pose.position.y+320
    #     self.x_data, self.y_data = [] , []
    #     self.y_data.append(self.y)
    #     self.x_data.append(self.x)
    #     fig.canvas.flush_events()

    # def gps_callback0(self, msg):
    #     print("Callback2")
    #     gps_state = msg.data
    #     if gps_state==1:
    #         self.gps_color0 = 'green'
    #     else:
    #         self.gps_color0 = 'red'

    # def gps_callback1(self, msg):
    #     gps_state = msg.data
    #     if gps_state==1:
    #         self.gps_color1 = 'green'
    #     else:
    #         self.gps_color1 = 'red'

    # def gps_callback2(self, msg):
    #     gps_state = msg.data
    #     if gps_state==1:
    #         self.gps_color2 = 'green'
    #     else:
    #         self.gps_color2 = 'red'

    def update_plot1b(self, frame):
        self.ln1b.remove()
        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw*180/3.1415-90), markersize=10, color='blue')
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
        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw*180/3.1415-90), markersize=10, color='blue')
        self.ln2y, = self.ax2.plot([], [], 'k-')
        self.ln2t, = self.ax2.plot([], [], 'k-')
        self.ln2u, = self.ax2.plot([], [], 'm-')
        self.ax2.axis([self.x-50, self.x+50, self.y-50, self.y+50])
        #self.ax2.invert_xaxis()
        self.ln2b.set_data(self.x_data, self.y_data)
        #colors
        fig_width, fig_height = plt.gcf().get_size_inches()
        self.ax2.text(0, 1.1, r'GPS:{}'.format(self.gps_code), transform=self.ax2.transAxes, horizontalalignment='left', size=20, color=self.gps_code_color) # fig_width*0.15 , 0.9 0.85 ,0.8
        self.ax2.text(0, 1.05, r'GPS uncertainty:{}'.format(self.gps_uncertainty, 3), transform=self.ax2.transAxes, horizontalalignment='left', size=20, color='black')
        self.ax2.text(0, 1, r'Speed[mps]:{}'.format(self.speed), transform=self.ax2.transAxes, horizontalalignment='left', size=20, color='black')
        if self.gps_code_color == 'red':
            self.fig.patch.set_facecolor('coral')
        elif self.gps_code_color == 'orange':
            self.fig.patch.set_facecolor('yellow')
        else:
            self.fig.patch.set_facecolor('white')

        legned = self.ax1.legend([self.ln2i,self.ln2o,self.ln2y,self.ln2p,self.ln2u], ['Inner lane', 'Outer lane', 'Pit lane', 'Pediction path', 'Waypoint'],bbox_to_anchor=(1.05, 0.0), ncol=2, fontsize = 10, frameon=False)
        # if self.gps_color0 == 'green' and self.gps_color1 == "green" and self.gps_color2 == 'green':
        #     self.fig.patch.set_facecolor('white')
        # elif self.gps_color0 == 'red' or self.gps_color1 == "red" or self.gps_color2 == 'red':
        #     self.fig.patch.set_facecolor('coral')
        # elif self.gps_color0 == 'black' or self.gps_color1 == "black" or self.gps_color2 == 'black':
        #     self.fig.patch.set_facecolor('orange')
        self.fig.patch.set_alpha(0.5)
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
    def update_plot2u(self, frame):
        self.ln2u.set_data(self.x_path_data, self.y_path_data)
        return self.ln2u                
fig, (ax1, ax2) = plt.subplots(2,1,gridspec_kw={'height_ratios': [1, 6]})

vis = Visualiser()
ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b)
ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i)
ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o)
ani1y = FuncAnimation(vis.fig, vis.update_plot1y, init_func=vis.plot_init1y)
ani1t = FuncAnimation(vis.fig, vis.update_plot1t, init_func=vis.plot_init1t)
ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b)
ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i)
ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o)
ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p)
ani2y = FuncAnimation(vis.fig, vis.update_plot2y, init_func=vis.plot_init2y)
ani2t = FuncAnimation(vis.fig, vis.update_plot2t, init_func=vis.plot_init2t)
ani2u = FuncAnimation(vis.fig, vis.update_plot2u, init_func=vis.plot_init2u)

vis.ax1.axis('scaled')
vis.ax2.axis('scaled')
vis.ax1.axis('off')
vis.ax2.axis('off')
plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
fig.show()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('Visualiser')
    #is = Visualiser()
    #sub_odom = node.create_subscription(Odometry, '/BLIO/odom_gicp_global_prediction', vis.odom_callback, 10)
    #sub_steer = node.create_subscription(Float32, '/steering', vis.steering_callback, 10)
    #sub_sensor0 = node.create_subscription(Bool, '/GPS0', vis.gps_callback0, 10)
    #sub_sensor1 = node.create_subscription(Bool, '/GPS1', vis.gps_callback1, 10)
    #sub_sensor2 = node.create_subscription(Bool, '/GPS2', vis.gps_callback2, 10)
    sub_odom = node.create_subscription(Telemetry, '/nif_telemetry/telemetry', vis.tele_callback, qos_profile_sensor_data)
    sub_path = node.create_subscription(Path, '/nif_telemetry/path_global', vis.path_callback, qos_profile_sensor_data)
    


    # print("spin_b")

    rclpy.spin(node)
    # print("spin_a")

    #node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
