#!/usr/bin/python
import argparse
import sys
import os
import rospy
import rosbag

# add time stamp to python recorded images in bag file

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''bag file to txt file''')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', help='out bag file')
    args = parser.parse_args()

    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag

    # inbag = rosbag.Bag(args.inputbag,'r')

    with rosbag.Bag(args.outputbag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(args.inputbag).read_messages():
            if topic == "/camera/rgb/image_color":
                msg.header.stamp = t
                outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)