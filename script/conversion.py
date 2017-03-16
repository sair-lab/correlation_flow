#!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import scipy
import tf
roll = 1
pitch =1
yaw = 0
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

q1 =  np.array([quaternion[0],quaternion[1], quaternion[2],quaternion[3]])

# m1 = tf.transformations.quaternion_matrix(quaternion)
# m2 = tf.transformations.quaternion_matrix(quaternion)

q2 = tf.transformations.quaternion_inverse(quaternion)


q3 = tf.transformations.quaternion_multiply(q1,q2)

print q3

euler = tf.transformations.euler_from_quaternion(quaternion)

print euler

