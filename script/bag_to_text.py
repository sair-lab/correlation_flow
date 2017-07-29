#!/usr/bin/python
import argparse
import sys
import os
import rospy
import rosbag
import tf

if __name__ == '__main__':
    print 1
    # parse command line
    parser = argparse.ArgumentParser(description='''bag file to txt file''')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputtxt', help='out text file')
    args = parser.parse_args()

    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputtxt

    inbag = rosbag.Bag(args.inputbag,'r')
    outtxt = open(args.outputtxt,'w')
    outtxt.write('# text file for '+ args.inputbag + '\n# format: time stamp x y z qx qy qz qw\n')

    for topic, msg, t in inbag.read_messages():
        if topic == "/px4flow/opt_flow":
            # q = (msg.pose.orientation.x,
            #     msg.pose.orientation.y,
            #     msg.pose.orientation.z,
            #     msg.pose.orientation.w)
            # yaw = tf.transformations.euler_from_quaternion(q)[2]
            outtxt.write(str.format("{0:.9f} ", t.to_sec()))
            outtxt.write(str.format("{0:.9f} ", msg.velocity_x))
            outtxt.write(str.format("{0:.9f} ", msg.velocity_y))
            # outtxt.write(str.format("{0:.9f} ", yaw))
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0\n')
            # outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.x))
            # outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.y))
            # outtxt.write(str.format("{0:.9f} ", msg.pose.orientation.z))
            # outtxt.write(str.format("{0:.9f}\n", msg.pose.orientation.w))

# topic = "/px4flow/opt_flow"
# header ground_distance flow_x flow_y velocity_x velocity_y quality
# topic = "/vicon_xb/viconPoseTopic"
# header pose.position(x, y, z) pose.orientation(x, y, z, w) vel(x, y, z)
# topic = "/correlation_flow_node/corrFlow_velocity"
# msg.twist.linear.x msg.twist.linear.y

# use t.to_sec() or msg.header.stamp.to_sec() ???

# euler = tf.transformations.euler_from_quaternion(quaternion)
