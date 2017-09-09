#!/usr/bin/env python
# license removed for brevity

import rospy
from vicon_xb.msg import viconPoseMsg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
# from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
import matplotlib
import numpy as np, time
from matplotlib import pyplot as plt
matplotlib.interactive(True)


window_width = 60
fig, ax = plt.subplots()
plotted_data, = ax.plot([], [], 'b-', lw = 1)

timer = []
vc_vx = []
cf_vx = []
vc_vy = []
cf_vy = []
vc_vz = []
cf_vz = []
t=0

# ax.set_xlim([0, window_width])
ax.set_ylim([-1.5, 1.5])

    
def velocity_estimation(twist, vicon):
    global time, data, n, window_width, plt,t

    t = twist.header.stamp.to_sec();

    dvx = twist.twist.linear.x * vicon.pose.position.z - vicon.vel.x;
    dvy = twist.twist.linear.y * vicon.pose.position.z - vicon.vel.y;
    dvz = twist.twist.linear.z * vicon.pose.position.z - vicon.vel.z;
    # print dvx, dvy, dvz

    # if timer==[]:
    #     if t < timer[-1]:
    #         timer = []
    #         data = []


    timer.append(t)
    vc_vx.append(-vicon.vel.x)
    cf_vx.append(twist.twist.linear.x * vicon.pose.position.z)
    vc_vy.append(vicon.vel.y)
    cf_vy.append(twist.twist.linear.y * vicon.pose.position.z)
    vc_vz.append(-vicon.vel.z)
    cf_vz.append(twist.twist.linear.z * vicon.pose.position.z)


if __name__ == '__main__':

    rospy.init_node('compare_node', anonymous=True)

    tss = ApproximateTimeSynchronizer([Subscriber("/correlation_flow_node/vision_speed/speed_twist", TwistStamped), 
                                       Subscriber("/vicon_xb/viconPoseTopic", viconPoseMsg)],10,0.1)
    tss.registerCallback(velocity_estimation)

    while not rospy.is_shutdown():
        if timer==[]:
            continue

        ax.plot(timer,vc_vx,'b-', lw = 1)
        ax.plot(timer,cf_vx,'r-', lw = 1)


        if t-timer[0] < window_width:
            ax.set_xlim([t-window_width, t])
        else:
            ax.set_xlim([t-window_width, t])


        time.sleep(0.01)
        fig.canvas.draw()