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
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from vfhp_local_planner.srv import SetGoal, SetGoalResponse
from vfhp_local_planner.msg import Histogram
from std_srvs.srv import Empty, EmptyResponse
import Planners.VFHP as vfhp

class VFHPNode(object):

    def __init__(self):
        rospy.init_node('vfhp_planner', log_level=rospy.DEBUG)

        #### Obtener parámetros
        self.__init_params()

        #### Inicializar atributos
        self.__init_attr()

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.active_window_pub = rospy.Publisher('vfhp/active_window', OccupancyGrid, queue_size=5)
        self.obstacle_grid_pub = rospy.Publisher('vfhp/obstacle_grid', OccupancyGrid, queue_size=5)
        self.polar_hist_pub = rospy.Publisher('vfhp/polar_hist', numpy_msg(Histogram), queue_size=5)
        #self.map_pub = rospy.Publisher('obstacle_grid', data_class, queue_size=3)

        # Subscribers
        self.TfBuffer = tf2_ros.Buffer(cache_time = rospy.Duration(secs=0.1))
        self.TfListener = tf2_ros.TransformListener(self.TfBuffer)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.laser_front_sub = rospy.Subscriber('scan_front', numpy_msg(LaserScan), self.laser_callback, queue_size=1)
        self.laser_back_sub = rospy.Subscriber('scan_back', numpy_msg(LaserScan), self.laser_callback, queue_size=1)
        #self.pose_sub = rospy.Subscriber('pose_kinematic', Pose2D, self.pose_callback)

        # Services
        self.goal_srv = rospy.Service("set_goal", SetGoal, self.set_goal_callback)

        self.plotter = None
        if (self.graphics):
            from Planners.plotter import Plotter
            self.plotter = Plotter(self.planner)
            self.draw_srv = rospy.Service("draw_hist", Empty, self.draw_hist_callback)


        #rospy.on_shutdown(self.draw_graphics)

    def __init_params(self):
        ## Parámetros ROS

        self.odom_frame_id = rospy.get_param('~odom_frame_id', default='odom')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', default='mecanum_base')
        self.graphics = rospy.get_param('~graphics', default=True)
        self.DECAY_RATE = rospy.get_param('~decay_rate', default=50)
        self.DECAY_VALUE = rospy.get_param('~decay_value', default=1)
        self.DECAY_GUARDBAND = rospy.get_param('~decay_guardband', default=3)

        ## Parámetros propios del algoritmo VFH+
        ## Para más información ver documentación del módulo VFH
        self.vparams = vfhp.VConst()
        self.vparams.GRID_SIZE = rospy.get_param('~grid_size', default=125)
        self.vparams.C_MAX = rospy.get_param('~c_max', default=20)
        self.vparams.RESOLUTION = rospy.get_param('~resolution', default=0.15)
        self.vparams.WINDOW_SIZE = rospy.get_param('~window_size', default=25)
        self.vparams.HIST_SIZE = rospy.get_param('~hist_size', default=180)
        self.vparams.B = rospy.get_param('~kb', default=10.0)
        self.vparams.D = rospy.get_param('~kd', default=1.0)
        self.vparams.E = rospy.get_param('~ke' ,default=2.0)
        self.vparams.R_ROB = rospy.get_param('~robot_radius', default=0.478)
        self.vparams.D_S = rospy.get_param('~d_s', default=0.05)
        self.vparams.T_LO = rospy.get_param('~t_lo', default=175000.0)
        self.vparams.T_HI = rospy.get_param('~t_hi', default=200000.0)
        self.vparams.V_MAX = rospy.get_param('~v_max', default=0.41)
        self.vparams.V_MIN = rospy.get_param('~v_min', default=0.0)
        self.vparams.MU1 = rospy.get_param('~mu1', default=6.0)
        self.vparams.MU2 = rospy.get_param('~mu2', default=2.0)
        self.vparams.MU3 = rospy.get_param('~mu3', default=2.0)
        self.vparams.DIST_FCN = rospy.get_param('~dist_fcn', default="GAUSS")
        self.vparams.update()

    def __init_attr(self):

        self.goal_reached = True
        self.goal = np.zeros(2,dtype=np.float64)
        self.goal_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        ## Usar?
        self.X_BIAS = self.vparams.GRID_SIZE*self.vparams.RESOLUTION/2.0
        self.Y_BIAS = self.vparams.GRID_SIZE*self.vparams.RESOLUTION/2.0

        self.planner = vfhp.VFHPModel(self.vparams)

        self.planner.update_position(self.X_BIAS, self.Y_BIAS, 0.0)

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
        rospy.logdebug_throttle(1, "Orientation (w, x, y, z): %.2f, %.2f, %.2f, %.2f" % (msg.pose.pose.orientation.w, 
                                                                                   msg.pose.pose.orientation.x,
                                                                                   msg.pose.pose.orientation.y, 
                                                                                   msg.pose.pose.orientation.z))
        self.planner.update_position(x + self.X_BIAS, y + self.Y_BIAS, yaw)



    def laser_callback(self, msg):


        # F1Tenth hotfix
        laser_frame = "{:s}_model".format(msg.header.frame_id)
        tf_found = self.TfBuffer.can_transform(self.robot_frame_id, laser_frame, rospy.Time(0))

        if not tf_found:
            rospy.logerr("Laser Transform not found from frame %s to target %s" %(laser_frame, self.robot_frame_id))

        else:

            # TODO:
            # Si las transformaciones son fijas se podrían buscar una única vez,
            # la primera vez que se recibe un mensaje, y guardar la info.

            tf_msg = self.TfBuffer.lookup_transform(self.robot_frame_id, laser_frame, rospy.Time.now())

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

            # rospy.logdebug_throttle(1, "Laser tf for %s\n%s\nSensor pos: %s\nreadings %s" %(laser_frame, tf_msg, str(trans), str(valid_data)))
            # rospy.logdebug_throttle(1, "Received LaserScan msg\n%s" % str(msg))

            # rospy.logdebug_throttle(1, "Transformation for {:s} frame to robot is (x, y, theta): ({:.2f}, {:.2f}, {:.3f})".format(laser_frame, trans[0], trans[0], euler[2]))
            # if msg.header.frame_id == "laser_front": rospy.logdebug_throttle(1, "Processed LaserScan msg from frame %s\nTotal readings: %d, discarded: %d\n" % (msg.header.frame_id, len(ranges), raw_data.shape[0]-valid_data.shape[0]))

            # XXX: Critical section START
            self.obstacle_lock.acquire()
            self.planner.update_obstacle_density(valid_data, trans)
            self.obstacle_lock.release()
            # XXX: Critical section END

    def pose_callback(self, msg):
        rospy.logdebug_throttle(1, "Received Pose2D message: %.2f, %.2f, %.2f" % (msg.x , msg.y, msg.theta) )
        self.planner.update_position(msg.x + self.X_BIAS, msg.y + self.Y_BIAS, msg.theta)

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
        # msg.linear.x = v*math.cos(theta - self.planner.cita)
        # msg.linear.y = v*math.sin(theta - self.planner.cita)
        # msg.angular.z = 0.0
        # XXX: HOTFIX for F1Tenth, usar marco de ref global
        msg.linear.x = v*math.cos(theta)
        msg.linear.y = v*math.sin(theta)
        self.cmd_pub.publish(msg)
        return


    def pub_active_window(self):

        msg = OccupancyGrid()


        msg.data = ((100/self.planner.const.C_MAX)*self.planner._active_grid().T.flatten()).tolist()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame_id
        msg.info.map_load_time = msg.header.stamp
        msg.info.height = self.planner.const.WINDOW_SIZE
        msg.info.width = self.planner.const.WINDOW_SIZE
        

        origin_d = (self.planner.const.WINDOW_SIZE-1)/2
        msg.info.origin.position.x = (self.planner.i_0 - origin_d)*self.planner.const.RESOLUTION - self.X_BIAS
        msg.info.origin.position.y = (self.planner.j_0 - origin_d)*self.planner.const.RESOLUTION - self.Y_BIAS
        msg.info.resolution = self.planner.const.RESOLUTION
        self.active_window_pub.publish(msg)

    def pub_obstacle_grid(self):
        msg = OccupancyGrid()



        msg.data = ((100/self.planner.const.C_MAX)*self.planner.obstacle_grid.T.flatten()).tolist()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame_id
        msg.info.map_load_time = msg.header.stamp
        msg.info.height = self.planner.const.GRID_SIZE
        msg.info.width = self.planner.const.GRID_SIZE

        # origin_d = (self.planner.const.GRID_SIZE-1)*self.planner.const.RESOLUTION/2.0
        msg.info.origin.position.x = - self.X_BIAS
        msg.info.origin.position.y = - self.Y_BIAS
        msg.info.resolution = self.planner.const.RESOLUTION
        self.obstacle_grid_pub.publish(msg)

    def pub_polar_hist(self):

        msg = Histogram()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame_id
        msg.size = len(self.planner.polar_hist)
        msg.data = self.planner.polar_hist

        msg.threshold_lower = self.planner.const.T_LO
        msg.threshold_upper = self.planner.const.T_HI
        self.polar_hist_pub.publish(msg)



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

            self.pub_active_window()
            self.pub_obstacle_grid()
            self.pub_polar_hist()

            if it  > 100:

                # XXX: Critical section START
                # rospy.logwarn_throttle(1, "decay was called")
                self.obstacle_lock.acquire()
                self.planner.decay_active_window(self.DECAY_VALUE, self.DECAY_GUARDBAND)
                self.obstacle_lock.release()
                # XXX: Critical section End

                it -= 100

            it += self.DECAY_RATE

            rate.sleep()

    def draw_graphics(self):
        if self.plotter is not None:
            self.plotter.plot_active_grid(1)
            self.plotter.plot_active_window(2)
            self.plotter.plot_hist(3)
            self.plotter.plot_show()


def main():
    node = VFHPNode()
    node.run()

if __name__ == '__main__':
    main()
