#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import scipy

rospy.init_node('VideoPublisher',anonymous=False)
VideoRaw = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
rate = rospy.Rate(50)


"""
#publish images from a camera
#----------------------------
cam = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    meta, frame = cam.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    msg_frame = CvBridge().cv2_to_imgmsg(frame, "mono8")
    msg_frame.header.stamp = rospy.Time.now()
    VideoRaw.publish(msg_frame)
    # cv2.imshow('frame', frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break
    rate.sleep()
cam.release()
#----------------------------


"""
#publish rotated images from a single image
#------------------------------------------
img = cv2.imread('/home/jeffsan/drones/src/correlation_flow/script/panda3.jpg')
dst = img
height, width = img.shape[:2]
#img = cv2.resize(img, (360, 240))
x, y = scipy.mgrid[-height/2: height/2 , -width/2: width/2 ]
g = scipy.exp(- (x ** 2/float(height*20) + y ** 2 / float(width*20)))
i = 0
while not rospy.is_shutdown():

#    if i==0:
#        msg_frame = CvBridge().cv2_to_imgmsg(img, "bgr8")
#    else:
#        M = np.float32([[1,0,2*i],[0,1,5]])
#        img = cv2.warpAffine(img,M,(360,240))
#        msg_frame = CvBridge().cv2_to_imgmsg(img, "bgr8")

    M = cv2.getRotationMatrix2D((width/2,height/2), i, 1+0.02*i)
    dst = cv2.warpAffine(dst,M,(width,height),flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,  borderValue=(127, 127, 127))
    # dst1 = np.float32(img)
    # dst1[:,:,0] *= g 
    # dst1[:,:,1] *= g 
    # dst1[:,:,2] *= g 
    # img = np.uint8(dst)
    msg_frame = CvBridge().cv2_to_imgmsg(dst, "bgr8")
    msg_frame.header.stamp=rospy.Duration(0.03*i)

    VideoRaw.publish(msg_frame)
    cv2.imshow('frame', dst)
    i = i+1
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break
    rate.sleep()
#-------------------------------------------


cv2.destroyAllWindows()

