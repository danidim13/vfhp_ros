#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

import tf
import tf2_ros

import rospy

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import *
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import Planners.VFHP as vfhp

class VFHPNode(object):

    def __init__(self):
        rospy.init_node('vfhp_planner', log_level=rospy.DEBUG)

        #### Obtener parámetros

        ## Parámetros ROS

        self.world_frame_id = rospy.get_param("~world_frame_id", default="world")
        self.robot_frame_id = rospy.get_param("~robot_frame_id", default="mecanum_base")
        self.DECAY_RATE = rospy.get_param("~decay_rate", default=90)

        self.goal_reached = True
        self.goal = np.zeros(2,dtype=np.float64)

        ## Parámetros propios del algoritmo VFH+
        ## Para más información ver documentación del módulo VFH
        self.grid_size = rospy.get_param('~grid_size', default=125)
        self.c_max = rospy.get_param('~c_max', default=20)
        self.resolution = rospy.get_param('~resolution', default =0.15)
        self.window_size = rospy.get_param('~window_size', default=25)
        self.window_center = self.window_size/2
        self.alpha = rospy.get_param('~alpha', default=5)
        self.hist_size = 360/self.alpha
        self.d_max2 = np.square((self.window_size-1)*self.resolution)/2.0
        self.kb = np.float_(10.0)
        self.ka = np.float_(1+self.kb*self.d_max2)
        self.r_rob = rospy.get_param('~robot_radius', default=0.755)
        self.d_s = rospy.get_param('~d_s',default=0.05)
        self.r_rs = self.r_rob + self.d_s
        self.t_lo = rospy.get_param('~t_lo',default=3000.0)
        self.t_hi = rospy.get_param('~t_hi', default=3500.0)
        self.wide_v = self.hist_size/8
        self.v_max = rospy.get_param('~v_max', default=0.31)
        self.v_min = rospy.get_param('~v_min', default=0.0)
        self.mu1 = rospy.get_param('~mu1', 6.0)
        self.mu2 = rospy.get_param('~mu2', 2.0)
        self.mu3 = rospy.get_param('~mu3', 2.0)
        self.max_cost = 180.0*(self.mu1 + self.mu2 + self.mu3)

        vfhp.GRID_SIZE = self.grid_size
        vfhp.C_MAX = self.c_max
        vfhp.RESOLUTION = self.resolution
        vfhp.WINDOW_SIZE = self.window_size
        vfhp.WINDOW_CENTER = self.window_center
        vfhp.ALPHA = self.alpha
        vfhp.HIST_SIZE = self.hist_size
        vfhp.D_max2 = self.d_max2
        vfhp.B = self.kb
        vfhp.A = self.ka
        vfhp.R_ROB = self.r_rob
        vfhp.T_LO = self.t_lo
        vfhp.T_HI = self.t_hi
        vfhp.WIDE_V = self.wide_v
        vfhp.V_MAX = self.v_max
        vfhp.V_MIN = self.v_min
        vfhp.mu1 = self.mu1
        vfhp.mu2 = self.mu2
        vfhp.mu3 = self.mu3
        vfhp.MAX_COST = self.max_cost

        ## Usar?
        self.X_BIAS = self.grid_size*self.resolution/2.0
        self.Y_BIAS = self.grid_size*self.resolution/2.0

        self.planner = vfhp.VFHPModel()

        self.planner.update_position(self.X_BIAS, self.Y_BIAS, 0.0)

        # Subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.laser_front_sub = rospy.Subscriber('scan_front', numpy_msg(LaserScan), self.laser_callback)
        self.laser_back_sub = rospy.Subscriber('scan_back', numpy_msg(LaserScan), self.laser_callback)
        self.pose_sub = rospy.Subscriber('pose_kinematic', Pose2D, self.pose_callback)

        self.TfBuffer = tf2_ros.Buffer(cache_time = rospy.Duration(secs=5))
        self.TfListener = tf2_ros.TransformListener(self.TfBuffer)

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.map_pub = rospy.Publisher('obstacle_grid', data_class, queue_size=3)

        rospy.on_shutdown(self.planner._plot_active)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation)
        theta = angles[2]
        rospy.logdebug_throttle(1, "Received Odom msg (x, y, cita): %.2f, %.2f, %.2f" % (x, y, theta))


    def laser_callback(self, msg):


        tf_found = self.TfBuffer.can_transform(self.robot_frame_id, msg.header.frame_id, rospy.Time(0))

        if not tf_found:
            rospy.logerr("Laser Transform not found from frame %s to target %s" %(msg.header.frame_id, self.robot_frame_id))

        else:

            # TODO:
            # Si las transformaciones son fijas se podrían buscar una única vez,
            # la primera vez que se recibe un mensaje, y guardar la info.

            tf_msg = self.TfBuffer.lookup_transform(self.robot_frame_id, msg.header.frame_id, rospy.Time(0))

            quat = np.array([tf_msg.transform.rotation.x,
                                tf_msg.transform.rotation.y,
                                tf_msg.transform.rotation.z,
                                tf_msg.transform.rotation.w], np.float64)

            euler = tf.transformations.euler_from_quaternion(quat)
            trans = np.array([tf_msg.transform.translation.x, tf_msg.transform.translation.y])

            ranges = msg.ranges
            angles = np.array([ euler[2] + msg.angle_min + i*msg.angle_increment for i in xrange(len(ranges))])
            raw_data = np.column_stack((ranges,angles))
            valid_data = raw_data[(raw_data[:,0] > msg.range_min) & (raw_data[:,0] < msg.range_max)]

            #rospy.logdebug_throttle(1, "Laser tf for %s\n%s\nSensor pos: %s\nreadings %s" %(msg.header.frame_id, tf_msg, str(trans), str(valid_data)))
            #rospy.logdebug_throttle(1, "Received LaserScan msg\n%s" % str(msg))
            #if msg.header.frame_id == "laser_front": rospy.logdebug_throttle(1, "Processed LaserScan msg from frame %s\nTotal readings: %d, discarded: %d\n" % (msg.header.frame_id, len(ranges), raw_data.shape[0]-valid_data.shape[0]))

            self.planner.update_obstacle_density(valid_data, trans)

    def pose_callback(self, msg):
        rospy.logdebug_throttle(1, "Received Pose2D message: %.2f, %.2f, %.2f" % (msg.x , msg.y, msg.theta) )
        self.planner.update_position(msg.x + self.X_BIAS, msg.y + self.Y_BIAS, math.degrees(msg.theta))

    def pub_cmd_vel(self, theta, v):
        # TODO:
        # Definir como se va a manejar la publicación de mensajes
        pass

    def run(self):

        rate = rospy.Rate(10)

        # Aprox cada 0.5 s
        it = 0
        while not rospy.is_shutdown():

            if not self.goal_reached:
                self.planner.update_active_window()
                self.planner.update_polar_histogram()
                self.planner.update_bin_polar_histogram()
                self.planner.update_masked_polar_hist(self.r_rob, self.r_rob)
                valles = self.planner.find_valleys()
                cita, v = self.planner.calculate_steering_dir(valles)
            else:
                cita = 0
                v = 0

            self.pub_cmd_vel(cita, v)

            if it  > 100:
                self.planner.decay_active_window(1, 3)
                it -= 100
                
            it += self.DECAY_RATE

            rate.sleep()




def main():
    node = VFHPNode()
    node.run()

if __name__ == '__main__':
    main()


        # Subscribers

        # Publishers
