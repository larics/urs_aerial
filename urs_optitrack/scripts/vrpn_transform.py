#!/usr/bin/env python

__author__ = 'thaus'

import rospy
import copy
import math
#from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import Float32

class VrpnTransform:

    def __init__(self):
        '''
        Initialization of the class.
        '''
        rospy.Subscriber('/vrpn_client_node/parrot/pose', PoseStamped, self.pose_cb)
        
        self.pose_pub = rospy.Publisher('Optitrack', PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher('Optitrack_vel', TwistStamped, queue_size=1)
        self.first_pass = True
        self.t_old = rospy.Time.now()

        self.tranform_matrix = []

        self.pos = Vector3()
        self.pos_old = Vector3()
        self.vel = Vector3()
        self.vel_old = Vector3()
        self.quat = Quaternion()
        self.euler = Quaternion()

        self.filt_const = 0.75

    def run(self):

        rospy.spin()
       

    def pose_cb(self, msg):
        #self.pos.x = -msg.pose.position.z
        #self.pos.y = -msg.pose.position.x
        #self.pos.z = msg.pose.position.y
        self.pos.x = msg.pose.position.x
        self.pos.y = -msg.pose.position.z
        self.pos.z = msg.pose.position.y

        self.quat.x = msg.pose.orientation.x
        self.quat.y = msg.pose.orientation.y
        self.quat.z = msg.pose.orientation.z
        self.quat.w = msg.pose.orientation.w

        if self.first_pass == True:
            self.first_pass = False
            self.pos_old = copy.deepcopy(self.pos)
            self.t_old = msg.header.stamp
        else:
            # compute velocity
            dt = msg.header.stamp.to_sec() - self.t_old.to_sec()
            self.t_old = msg.header.stamp
            dt = 0.01

            if dt < 0.001:
                dt = 0.001

            self.vel.x = self.filt_const * (self.pos.x - self.pos_old.x ) / dt + (1.0 - self.filt_const) * self.vel_old.x
            self.vel.y = self.filt_const * (self.pos.y - self.pos_old.y ) / dt + (1.0 - self.filt_const) * self.vel_old.y
            self.vel.z = self.filt_const * (self.pos.z - self.pos_old.z ) / dt + (1.0 - self.filt_const) * self.vel_old.z

            self.vel_old = copy.deepcopy(self.vel)
            self.pos_old = copy.deepcopy(self.pos)
            
            # quaternion to euler
            qx = self.quat.x
            qy = self.quat.y
            qz = self.quat.z
            qw = self.quat.w
            self.euler.x = 0*math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
            self.euler.z = -math.asin(2*(qx * qz - qw * qy))
            self.euler.y = 0*math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.pose.position = copy.deepcopy(self.pos)
            pose_msg.pose.orientation = copy.deepcopy(self.euler)

            vel_msg = TwistStamped()
            vel_msg.header.stamp = msg.header.stamp
            vel_msg.twist.linear = copy.deepcopy(self.vel)

            self.pose_pub.publish(pose_msg)
            self.vel_pub.publish(vel_msg)


        

   
if __name__ == '__main__':

    rospy.init_node('vrpn_transformer')
    vrpn = VrpnTransform()
    vrpn.run()

