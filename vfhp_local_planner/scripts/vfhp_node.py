#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

import tf
import tf2_ros

import rospy

import threading

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mecanumrob_common.srv import SetGoal, SetGoalResponse
from std_srvs.srv import Empty, EmptyResponse
import Planners.VFHP as vfhp

class VFHPNode(object):

    def __init__(self):
        rospy.init_node('vfhp_planner', log_level=rospy.DEBUG)

        #### Obtener parámetros

        ## Parámetros ROS

        self.odom_frame_id = rospy.get_param('~odom_frame_id', default='odom')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', default='mecanum_base')
        self.DECAY_RATE = rospy.get_param('~decay_rate', default=50)


        self.goal_reached = True
        self.goal = np.zeros(2,dtype=np.float64)
        self.goal_lock = threading.Lock()

        self.obstacle_lock = threading.Lock()

        ## Parámetros propios del algoritmo VFH+
        ## Para más información ver documentación del módulo VFH
        self.vparams = vfhp.VConst()
        self.vparams.GRID_SIZE = rospy.get_param('~grid_size', default=125)
        self.vparams.C_MAX = rospy.get_param('~c_max', default=20)
        self.vparams.RESOLUTION = rospy.get_param('~resolution', default=0.15)
        self.vparams.WINDOW_SIZE = rospy.get_param('~window_size', default=25)
        self.vparams.ALPHA = rospy.get_param('~alpha', default=5)
        self.vparams.B = rospy.get_param('~kb', default=10.0)
        self.vparams.R_ROB = rospy.get_param('~robot_radius', default=0.478)
        self.vparams.D_S = rospy.get_param('~d_s', default=0.05)
        self.vparams.T_LO = rospy.get_param('~t_lo', default=175000.0)
        self.vparams.T_HI = rospy.get_param('~t_hi', default=200000.0)
        self.vparams.V_MAX = rospy.get_param('~v_max', default=0.41)
        self.vparams.V_MIN = rospy.get_param('~v_min', default=0.0)
        self.vparams.MU1 = rospy.get_param('~mu1', default=6.0)
        self.vparams.MU2 = rospy.get_param('~mu2', default=2.0)
        self.vparams.MU3 = rospy.get_param('~mu3', default=2.0)
        self.vparams.update()

        ## Usar?
        self.X_BIAS = self.vparams.GRID_SIZE*self.vparams.RESOLUTION/2.0
        self.Y_BIAS = self.vparams.GRID_SIZE*self.vparams.RESOLUTION/2.0

        self.planner = vfhp.VFHPModel(self.vparams)

        self.planner.update_position(self.X_BIAS, self.Y_BIAS, 0.0)

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.map_pub = rospy.Publisher('obstacle_grid', data_class, queue_size=3)

        # Subscribers
        self.TfBuffer = tf2_ros.Buffer(cache_time = rospy.Duration(secs=5))
        self.TfListener = tf2_ros.TransformListener(self.TfBuffer)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.laser_front_sub = rospy.Subscriber('scan_front', numpy_msg(LaserScan), self.laser_callback)
        self.laser_back_sub = rospy.Subscriber('scan_back', numpy_msg(LaserScan), self.laser_callback)
        #self.pose_sub = rospy.Subscriber('pose_kinematic', Pose2D, self.pose_callback)


        # Services
        self.goal_srv = rospy.Service("set_goal", SetGoal, self.set_goal_callback)
        self.draw_srv = rospy.Service("draw_hist", Empty, self.draw_hist_callback)

        #rospy.on_shutdown(self.draw_graphics)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        quat = np.array([msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w], np.float64)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        #theta = yaw
        rospy.logdebug_throttle(1, "Received Odom msg (x, y, cita): %.2f, %.2f, %.2f" % (x, y, yaw))
        self.planner.update_position(x + self.X_BIAS, y + self.Y_BIAS, math.degrees(yaw))



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

            self.obstacle_lock.acquire()
            self.planner.update_obstacle_density(valid_data, trans)
            self.obstacle_lock.release()

    def pose_callback(self, msg):
        rospy.logdebug_throttle(1, "Received Pose2D message: %.2f, %.2f, %.2f" % (msg.x , msg.y, msg.theta) )
        self.planner.update_position(msg.x + self.X_BIAS, msg.y + self.Y_BIAS, math.degrees(msg.theta))

    def set_goal_callback(self, req):
        if req.set:
            x_targ = req.x + self.X_BIAS
            y_targ = req.y + self.Y_BIAS

            if (x_targ < 0 or y_targ < 0
                    or self.vparams.RESOLUTION*self.vparams.GRID_SIZE < x_targ
                    or self.vparams.RESOLUTION*self.vparams.GRID_SIZE < y_targ):
                return SetGoalResponse(False, 'Target out of bounds')
            else:

                # XXX: Crital Section Start
                self.goal_lock.acquire()
                self.goal_reached = False
                self.goal[0] = req.x
                self.goal[1] = req.y
                self.planner.set_target(x_targ, y_targ)
                self.goal_lock.release()
                # XXX: Crital Section End

                return SetGoalResponse(True, '')

        else:
            self.goal_reached = True
            return SetGoalResponse(True, '')

    def draw_hist_callback(self, req):
        self.draw_graphics()
        return EmptyResponse()

    def check_goal_reached(self):


        # XXX: Crital Section Start
        self.goal_lock.acquire()

        if not self.goal_reached and self.planner.get_target_dist() < self.vparams.R_ROB/3:
                self.goal_reached = True

        self.goal_lock.release()
        # XXX: Crital Section End

        return


    def pub_cmd_vel(self, theta, v):
        # TODO:
        # Decidir que hacer con theta

        msg = Twist()
        msg.linear.x = v*math.cos(math.radians(theta) - math.radians(self.planner.cita))
        msg.linear.y = v*math.sin(math.radians(theta) - math.radians(self.planner.cita))
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        return

    def run(self):

        rate = rospy.Rate(10)

        # Aprox cada 0.5 s
        it = 0
        while not rospy.is_shutdown():

            self.planner.update_active_window()
            self.planner.update_polar_histogram()
            self.planner.update_bin_polar_histogram()
            self.planner.update_masked_polar_hist(0, 0, True)

            if not self.goal_reached:
                valles = self.planner.find_valleys()
                cita, v = self.planner.calculate_steering_dir(valles)
                self.check_goal_reached()
                rospy.logdebug_throttle(2, "Heading to %s at %s m/s." % (cita, v))
            else:
                rospy.logdebug_throttle(2, "No goal set.")
                cita = 0
                v = 0

            self.pub_cmd_vel(cita, v)

            if it  > 100:
                self.planner.decay_active_window(1, 3)
                it -= 100

            it += self.DECAY_RATE

            rate.sleep()

    def draw_graphics(self):
        self.planner._plot_active_grid(1)
        self.planner._plot_active_window(2)
        self.planner._plot_hist(3)
        self.planner._plot_show()






def main():
    node = VFHPNode()
    node.run()

if __name__ == '__main__':
    main()


        # Subscribers

        # Publishers
