#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import scipy

rospy.init_node('VideoPublisher',anonymous=False)
VideoRaw = rospy.Publisher('/camera/rgb/image_color', Image, queue_size=10)
rate = rospy.Rate(20)


"""
#publish images from a camera
#----------------------------
cam = cv2.VideoCapture(1)
while not rospy.is_shutdown():
    meta, frame = cam.read()
    msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
    VideoRaw.publish(msg_frame)q
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    rate.sleep()
cam.release()
#----------------------------
"""


#publish rotated images from a single image
#------------------------------------------
img = cv2.imread('/home/eee/drones/src/correlation_flow/script/panda3.jpg')
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

    M = cv2.getRotationMatrix2D((width/2,height/2), 1*i, 1)
    dst = cv2.warpAffine(img,M,(width,height),flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,  borderValue=(127, 127, 127))
    dst1 = np.float32(dst)
    # dst1[:,:,0] *= g 
    # dst1[:,:,1] *= g 
    # dst1[:,:,2] *= g 
    dst = np.uint8(dst1)
    msg_frame = CvBridge().cv2_to_imgmsg(dst, "bgr8")

    VideoRaw.publish(msg_frame)
    cv2.imshow('frame', dst)
    i = i+1
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break
    rate.sleep()
#-------------------------------------------


cv2.destroyAllWindows()

