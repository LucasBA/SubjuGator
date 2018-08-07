#!/usr/bin/env python
import rospy
import tf.transformations
from mil_msgs.msg import DepthStamped, VelocityMeasurements, PoseTwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped, TwistWithCovarianceStamped


def got_depth(msg):
    depth_pub = rospy.Publisher('/ukf_depth', PoseWithCovarianceStamped, queue_size=1)
    p = PoseWithCovarianceStamped()
    p.header = msg.header
    # Create a new pose
    p.pose.pose.position.x = 0.0
    p.pose.pose.position.y = 0.0
    p.pose.pose.position.z = -msg.depth
    depth_pub.publish(p)


def got_mag(msg):
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


def got_dvl(msg):
    dvl_pub = rospy.Publisher('/ukf_dvl', TwistWithCovarianceStamped, queue_size=1)
    v = TwistWithCovarianceStamped()
    v.header = msg.header
    new_twist = dvl2twist(msg.velocity_measurements)
    v.twist.twist = new_twist
    dvl_pub.publish(v)


def got_trajectory(msg):
    trajectory_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
    v = TwistStamped()
    v.header = msg.header
    v.twist = msg.posetwist.twist
    trajectory_pub.publish(v)


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


if __name__ == '__main__':
    rospy.init_node('state_message_remaper')
    depth_sub = rospy.Subscriber('/depth', DepthStamped, got_depth)
    mag_sub = rospy.Subscriber('/mag_orientation', Vector3, got_mag)
    dvl_sub = rospy.Subscriber('/dvl', VelocityMeasurements, got_dvl)
    trajectory_sub = rospy.Subscriber('/trajectory', PoseTwistStamped, got_trajectory)

    rospy.spin()
