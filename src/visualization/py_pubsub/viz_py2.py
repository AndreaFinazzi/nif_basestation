import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
#import tf
from tf_transformations import quaternion_matrix
import tf_transformations
import numpy as np
from matplotlib.animation import FuncAnimation
import csv


class Visualiser(Node):
    def __init__(self):
        # super().__init__('Visualiser')
        # self.sub_odom = self.create_subscription(Odometry, '/BLIO/odom_gicp_global_prediction', self.odom_callback, 10)
        # self.sub_steer = self.create_subscription(Float32, '/steering', self.steering_callback, 10)
        # self.sub_sensor0 = self.create_subscription(Bool, '/GPS0', self.gps_callback0, 10)
        # self.sub_sensor1 =self.create_subscription(Bool, '/GPS1', self.gps_callback1, 10)
        # self.sub_sensor2 = self.create_subscription(Bool, '/GPS2', self.gps_callback2, 10)       
        self.range =np.pi
        self.x_pred=0
        self.y_pred=0
        self.incre=0.05
        self.steering=0
        self.radius=0
        self.wheelbase = 2.0
        self.max = 10**10
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2 
        self.x_data, self.y_data = [] , []
        self.yaw=0
        self.x=0
        self.y=0
        self.x_bias = 1600
        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw))
        self.ln1i, = self.ax1.plot([], [], 'r-')
        self.ln1o, = self.ax1.plot([], [], 'g-')
        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw))
        self.ln2i, = self.ax2.plot([], [], 'r-')
        self.ln2o, = self.ax2.plot([], [], 'g-')
        self.ln2p, = self.ax2.plot([], [], 'b-')
        self.x_data, self.y_data = [] , []
        self.x_in_data, self.y_in_data = [] , []
        self.x_out_data, self.y_out_data = [] , []
        self.x_pred_data, self.y_pred_data = [] , []
        self.cnt=0
        self.gps_color0 = 'black'
        self.gps_color1 = 'black'
        self.gps_color2 = 'black'


        f = open('/home/bang/nif/src/py_pubsub/py_pubsub/ims_inner_211004_cy_modified_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_in_data.append(float(line[0])+self.x_bias)
                self.y_in_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0
        f = open('/home/bang/nif/src/py_pubsub/py_pubsub/ims_outer_211004_cy_modified_wpt.csv','r')
        rdr = csv.reader(f)
        for line in rdr:
            if (self.cnt!=0):
                self.x_out_data.append(float(line[0])+self.x_bias)
                self.y_out_data.append(float(line[1]))
            self.cnt=1
        f.close()
        self.cnt=0

    def plot_init1b(self):
       
        self.ax1.set_xlim(-850,850)
        self.ax1.set_ylim(-850,850)
        self.ax1.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x))
        self.ax1.spines['top'].set_color('none')
        self.ax1.invert_xaxis()
        return self.ln1b

    def plot_init1i(self):
        self.ax1.set_xlim(-850,850)
        self.ax1.set_ylim(-850,850)
        self.ax1.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x))
        self.ax1.spines['top'].set_color('none')
        self.ax1.invert_xaxis()
        return self.ln1i

    def plot_init1o(self):
        self.ax1.set_xlim(-850,850)
        self.ax1.set_ylim(-850,850)
        self.ax1.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax1.spines['right'].set_color('none')
        self.ax1.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x_data))
        self.ax1.spines['top'].set_color('none')
        self.ax1.invert_xaxis()
        return self.ln1o

    def plot_init2b(self):
        self.ax2.set_xlim(-850,850)
        self.ax2.set_ylim(-850,850)
        self.ax2.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x_data))
        self.ax2.spines['top'].set_color('none')
        self.ax2.invert_xaxis()
        return self.ln2b    

    def plot_init2i(self):
        self.ax2.set_xlim(-850,850)
        self.ax2.set_ylim(-850,850)
        self.ax2.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x_data))
        self.ax2.spines['top'].set_color('none')
        self.ax2.invert_xaxis()
        return self.ln2i    

    def plot_init2o(self):
        self.ax2.set_xlim(-850,850)
        self.ax2.set_ylim(-850,850)
        self.ax2.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x_data))
        self.ax2.spines['top'].set_color('none')
        self.ax2.invert_xaxis()
        return self.ln2o    

    def plot_init2p(self):
        self.ax2.set_xlim(-850,850)
        self.ax2.set_ylim(-850,850)
        self.ax2.spines['left'].set_position('zero')
        #self.ax.spines['left'].set_position(('data',self.y))
        self.ax2.spines['right'].set_color('none')
        self.ax2.spines['bottom'].set_position('zero')
        #self.ax.spines['bottom'].set_position(('data',self.x_data))
        self.ax2.spines['top'].set_color('none')
        self.ax2.invert_xaxis()
        return self.ln2p    

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw  

    def steering_callback(self, msg):
        self.x_pred_data=[]
        self.y_pred_data=[]
        self.steering = msg.data
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

    def odom_callback(self, msg):
        print("Callback1")
        self.yaw = self.getYaw(msg.pose.pose)
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y+320
        self.x_data, self.y_data = [] , []
        self.y_data.append(self.y)
        self.x_data.append(self.x)
        #plt.get_current_fig_manager().show()
        #plt.ioff()
        #plt.draw()
        #plt.pause(0.001)  
        #ani1b = FuncAnimation(self.fig, self.update_plot1b, init_func=self.plot_init1b)
        #ani1i = FuncAnimation(self.fig, self.update_plot1i, init_func=self.plot_init1i)
        #ani1o = FuncAnimation(self.fig, self.update_plot1o, init_func=self.plot_init1o)
        #ani2b = FuncAnimation(self.fig, self.update_plot2b, init_func=self.plot_init2b)
        #ani2i = FuncAnimation(self.fig, self.update_plot2i, init_func=self.plot_init2i)
        #ani2o = FuncAnimation(self.fig, self.update_plot2o, init_func=self.plot_init2o)
        #ani2p = FuncAnimation(self.fig, self.update_plot2p, init_func=self.plot_init2p)
        #self.ax1.axis('scaled')
        #self.ax2.axis('scaled')
        #self.ax1.axis('off')
        #self.ax2.axis('off')
        #plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
        #plt.ioff()
        #plt.draw()
        #plt.pause(0.1)
        
        #plt.show()
        #plt.cla()         
        #plt.show(block=True)
        #plt.pause(0.1)
        #fig.show()
        
        #plt.remove()'
        #fig.canvas.draw_idle()
        #fig.canvas.draw()
        #plt.clf()
        fig.canvas.flush_events()

    def gps_callback0(self, msg):
        print("Callback2")
        gps_state = msg.data
        if gps_state==1:
            self.gps_color0 = 'green'
        else:
            self.gps_color0 = 'red'

    def gps_callback1(self, msg):
        gps_state = msg.data
        if gps_state==1:
            self.gps_color1 = 'green'
        else:
            self.gps_color1 = 'red'

    def gps_callback2(self, msg):
        gps_state = msg.data
        if gps_state==1:
            self.gps_color2 = 'green'
        else:
            self.gps_color2 = 'red'
        #plt.show(block=False)
    def update_plot1b(self, frame):
        #fig.clear()
        self.ln1b.remove()
        self.ln1b, = self.ax1.plot([], [], marker=(3, 0, self.yaw*180/3.1415), markersize=10, color='blue')
        self.ln1b.set_data(self.y_data, self.x_data)
        return self.ln1b

    def update_plot1i(self, frame):
        self.ln1i.set_data(self.y_in_data, self.x_in_data)
        return self.ln1i

    def update_plot1o(self, frame):
        self.ln1o.set_data(self.y_out_data, self.x_out_data)
        return self.ln1o

    def update_plot2b(self, frame):
        self.ln2b.remove()
        self.ax2.clear()
        #fig, (ax1, ax2) = plt.subplots(1,2)
        #self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw))
        self.ax2.axis('off')
        self.ax2.axis('scaled')
        self.ln2i, = self.ax2.plot([], [], 'r-')
        self.ln2o, = self.ax2.plot([], [], 'g-')
        self.ln2p, = self.ax2.plot([], [], 'b-')
        self.ln2b, = self.ax2.plot([], [], marker=(3, 0, self.yaw*180/3.1415), markersize=10, color='blue')

        self.ax2.axis([self.y-50, self.y+50, self.x-100, self.x+100])
        self.ax2.invert_xaxis()
        self.ln2b.set_data(self.y_data, self.x_data)
        #colors
        fig_width, fig_height = plt.gcf().get_size_inches()
        self.ax2.text(0, 0.9, r'GPS:{}'.format(0), transform=self.ax2.transAxes, horizontalalignment='center', size=20, color=self.gps_color0) # fig_width*0.15 , 0.9 0.85 ,0.8
        self.ax2.text(0, 0.85, r'GPS:{}'.format(1), transform=self.ax2.transAxes, horizontalalignment='center', size=20, color=self.gps_color1)
        self.ax2.text(0, 0.8, r'GPS:{}'.format(2), transform=self.ax2.transAxes, horizontalalignment='center', size=20, color=self.gps_color2)
        if self.gps_color0 == 'green' and self.gps_color1 == "green" and self.gps_color2 == 'green':
            self.fig.patch.set_facecolor('white')
        elif self.gps_color0 == 'red' or self.gps_color1 == "red" or self.gps_color2 == 'red':
            self.fig.patch.set_facecolor('coral')
        elif self.gps_color0 == 'black' or self.gps_color1 == "black" or self.gps_color2 == 'black':
            self.fig.patch.set_facecolor('orange')
        self.fig.patch.set_alpha(0.5)
        return self.ln2b

    def update_plot2i(self, frame):
        self.ln2i.set_data(self.y_in_data, self.x_in_data)
        return self.ln2i

    def update_plot2o(self, frame):
        self.ln2o.set_data(self.y_out_data, self.x_out_data)
        return self.ln2o

    def update_plot2p(self, frame):
        self.ln2p.set_data(self.y_pred_data, self.x_pred_data)
        return self.ln2p

# def main(args=None):
#     rclpy.init(args=args)
#     vis = Visualiser()
#     print("1")

#     ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b)
#     ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i)
#     ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o)
#     ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b)
#     ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i)
#     ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o)
#     ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p)
#     vis.ax1.axis('scaled')
#     vis.ax2.axis('scaled')
#     vis.ax1.axis('off')
#     vis.ax2.axis('off')
#     plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
#     plt.show(block=True)
#     plt.cla()

#     rclpy.spin(vis)
#     #vis.destroy_node()
#     #rclpy.shutdown()

# if __name__ == '__main__':
#   main()

# rclpy.create_node('lidar_visual_node')
# vis = Visualiser()
# sub = rclpy.Subscriber('/BLIO/odom_gicp_global_prediction', Odometry, vis.odom_callback)
# sub = rclpy.Subscriber('/GPS0', Bool, vis.gps_callback0)
# sub = rclpy.Subscriber('/GPS1', Bool, vis.gps_callback1)
# sub = rclpy.Subscriber('/GPS2', Bool, vis.gps_callback2)
# sub_steering = rclpy.Subscriber('/steering', Float32, vis.steering_callback)

# ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b)
# ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i)
# ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o)
# ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b)
# ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i)
# ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o)
# ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p)
# vis.ax1.axis('scaled')
# vis.ax2.axis('scaled')
# vis.ax1.axis('off')
# vis.ax2.axis('off')
# plt.subplots_adjust(left=0, bottom=0, right=1, top=1,
#                 wspace=0, hspace=0)
# plt.show(block=True)
# plt.cla()

#plt.ion()
fig, (ax1, ax2) = plt.subplots(1,2)

vis = Visualiser()
ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b)
ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i)
ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o)
ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b)
ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i)
ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o)
ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p)
vis.ax1.axis('scaled')
vis.ax2.axis('scaled')
vis.ax1.axis('off')
vis.ax2.axis('off')
plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
fig.show()
#fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('Visualiser')
    #is = Visualiser()
    sub_odom = node.create_subscription(Odometry, '/BLIO/odom_gicp_global_prediction', vis.odom_callback, 10)
    sub_steer = node.create_subscription(Float32, '/steering', vis.steering_callback, 10)
    sub_sensor0 = node.create_subscription(Bool, '/GPS0', vis.gps_callback0, 10)
    sub_sensor1 = node.create_subscription(Bool, '/GPS1', vis.gps_callback1, 10)
    sub_sensor2 = node.create_subscription(Bool, '/GPS2', vis.gps_callback2, 10) 
    #node = rclpy.create_node('Visualiser')


    print("init")

    print("sub_")

    #ani1b = FuncAnimation(vis.fig, vis.update_plot1b, init_func=vis.plot_init1b)
    #ani1i = FuncAnimation(vis.fig, vis.update_plot1i, init_func=vis.plot_init1i)
    #ani1o = FuncAnimation(vis.fig, vis.update_plot1o, init_func=vis.plot_init1o)
    #ani2b = FuncAnimation(vis.fig, vis.update_plot2b, init_func=vis.plot_init2b)
    #ani2i = FuncAnimation(vis.fig, vis.update_plot2i, init_func=vis.plot_init2i)
    #ani2o = FuncAnimation(vis.fig, vis.update_plot2o, init_func=vis.plot_init2o)
    #ani2p = FuncAnimation(vis.fig, vis.update_plot2p, init_func=vis.plot_init2p)
    #vis.ax1.axis('scaled')
    #vis.ax2.axis('scaled')
    #vis.ax1.axis('off')
    #vis.ax2.axis('off')
    #plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
    #plt.ion()
    #plt.draw()
    #plt.pause(0.1)    
    #plt.ion()
    #plt.show()
    #plt.show(block=True)
    #plt.show()
    #vis.fig.canvas.draw()
    #plt.show(block=False)    
    #plt.cla()
    #plt.pause(0.1)

    print("spin_b")

    rclpy.spin(node)
    print("spin_a")

    #node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
