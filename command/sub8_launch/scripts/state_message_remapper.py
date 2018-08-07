#!/usr/bin/env python
import rospy
import sys
import numpy as np
import tf.transformations
from mil_msgs.msg import DepthStamped, VelocityMeasurements, PoseTwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


def dvl2twist(velocity_measurements):
    x, y, z = 0, 0, 0
    for i in velocity_measurements:
        x = x + i.direction.x * i.velocity
        y = y + i.direction.y * i.velocity
        z = z + i.direction.z * i.velocity
    new_twist = Twist()
    new_twist.linear.x = x
    new_twist.linear.y = y
    new_twist.linear.z = z
    return new_twist


class Remapper:

    def __init__(self):
        rospy.sleep(1.0)
        self.initial_odom_check = True
        self.depth_sub = rospy.Subscriber('/depth', DepthStamped, self.got_depth)
        self.mag_sub = rospy.Subscriber('/mag_orientation', Vector3, self.got_mag)
        self.dvl_sub = rospy.Subscriber('/dvl', VelocityMeasurements, self.got_dvl)
        self.trajectory_sub = rospy.Subscriber('/trajectory', PoseTwistStamped, self.got_trajectory)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.got_odom)
        self.initial_odom_sub = rospy.Subscriber('/initial_odom', Odometry, self.got_initial_odom)


    def got_depth(self,msg):
        depth_pub = rospy.Publisher('/ukf_depth', PoseWithCovarianceStamped, queue_size=1)
        p = PoseWithCovarianceStamped()
        p.header = msg.header
        p.header.frame_id = "map"
        # Create a new pose
        p.pose.pose.position.x = 0.0
        p.pose.pose.position.y = 0.0
        p.pose.pose.position.z = -msg.depth
        p.pose.covariance = np.zeros(36) #+ 1e-2
        depth_pub.publish(p)


    def got_mag(self,msg):
        mag_pub = rospy.Publisher('/ukf_mag', PoseWithCovarianceStamped, queue_size=1)
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'imu'
        quat = tf.transformations.quaternion_from_euler(msg.x, msg.y, msg.z)
        p.pose.pose.orientation.x = quat[0]
        p.pose.pose.orientation.y = quat[1]
        p.pose.pose.orientation.z = quat[2]
        p.pose.pose.orientation.w = quat[3]
        mag_pub.publish(p)


    def got_dvl(self,msg):
        dvl_pub = rospy.Publisher('/ukf_dvl', TwistWithCovarianceStamped, queue_size=1)
        v = TwistWithCovarianceStamped()
        v.header = msg.header
        new_twist = dvl2twist(msg.velocity_measurements)
        v.twist.twist = new_twist
        dvl_pub.publish(v)


    def got_trajectory(self,msg):
        trajectory_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
        v = TwistStamped()
        v.header = msg.header
        v.twist = msg.posetwist.twist
        trajectory_pub.publish(v)


    def got_odom(self, msg):
        odom_pub = rospy.Publisher('/initial_odom', Odometry, queue_size=1)
        if (self.initial_odom_check):
            print "~~~~~~~~~~~~~~~~~~~PUBLISH ODOM~~~~~~~~~~~~~~~~~~"
            msg.pose.covariance = np.zeros(36)
            odom_pub.publish(msg)
            rospy.sleep(1.0)


    def got_initial_odom(self, msg):
        self.initial_odom_check = False

def main(args):
    Remapper()
    rospy.spin()

if __name__ == '__main__':
    print "~~~~~~~~~~~~~~~~~~~~~~~~REMAPPER~~~~~~~~~~~~~~~~~~~~~~~"
    rospy.init_node('state_message_remaper')
    main(sys.argv)
