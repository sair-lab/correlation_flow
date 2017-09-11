#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped


def callback(twist, pose):
    h = pose.pose.pose.position.z
    vx = twist.twist.linear.x * h
    vy = twist.twist.linear.y * h
    vz = twist.twist.linear.z * h
    yaw_rate = twist.twist.angular.z
    
    outtxt.write(str.format("{0:.9f} ", twist.header.stamp.to_sec()))
    outtxt.write(str.format("{0:.9f} ", vx))
    outtxt.write(str.format("{0:.9f} ", vy))
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0\n')


if __name__ == '__main__':

    outtxt = open('/home/jitete/drones/src/correlation_flow/bag/rect_cf.txt','w')
    outtxt.write('# text file' + '\n# format: time stamp x y z qx qy qz qw\n')

    rospy.init_node('msgfilter', anonymous=True)

    twist_sub = message_filters.Subscriber('/correlation_flow_node/vision_speed/speed_twist', TwistStamped)
    pose_sub = message_filters.Subscriber('/lidar_px4', PoseWithCovarianceStamped)

    ts = message_filters.ApproximateTimeSynchronizer([twist_sub, pose_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

