#!/usr/bin/python
import argparse
import sys
import os
import rospy
import rosbag

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
            outtxt.write(str.format("{0:.9f} ", t.to_sec()))
            outtxt.write(str.format("{0:.9f} ", msg.velocity_x))
            outtxt.write(str.format("{0:.9f} ", msg.velocity_y))
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0 ')
            outtxt.write('0\n')

# topic = "/px4flow/opt_flow"
# header ground_distance flow_x flow_y velocity_x velocity_y quality
# topic = "/vicon_xb/viconPoseTopic"
# header pose.position(x, y, z) pose.orientaion(x, y, z, w) vel(x, y, z)

# use t.to_sec() or msg.header.stamp.to_sec() ???
