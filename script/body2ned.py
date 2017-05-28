#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from px_comm.msg import OpticalFlow
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from numpy import *
import tf
import math

q = array([0,0,0,1])
v = array([0,0,0])
vx_p = 0
vy_p = 0

t = rospy.Time()

yaw = 130.0/180 * math.pi  # from NED to Vicon frame

def imucallback(msg):
    # attitude measured in ENU frame
    global q, t
    q = array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    t = msg.header.stamp

def toNED(msg):
    # fliter measured velocity
    global vx_p, vy_p

    vx = 1*msg.twist.linear.x + 0*vx_p
    vy = 1*msg.twist.linear.y + 0*vy_p
    v_body = array([vx, -vy, 0])
    
    # transform body velocity to ENU 
    global q
    [qx, qy, qz, qw] = [q[0], q[1], q[2], q[3]]
    Tenu = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])

    v = dot(Tenu, v_body)

    # ENU to NED: (x,y,z) -> (x,-y,-z)
    twist = TwistStamped()
    twist.header = Header()
    twist.header.frame_id = "ned"
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = v[0]
    twist.twist.linear.y = -v[1]
    twist.twist.linear.z = 0
    pub.publish(twist)

    vx_p = vx
    vy_p = vy

    # record data in vicon frame, compare with vicon
    q_ned_vicon = tf.transformations.quaternion_from_euler(math.pi, 0, -yaw)
    [qx, qy, qz, qw] = [q_ned_vicon[0], q_ned_vicon[1], q_ned_vicon[2], q_ned_vicon[3]]
    Tv = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
                  [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
                  [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    vr = dot(Tv, array([v[0],-v[1],0]))

    outtxt.write(str.format("{0:.9f} ", t.to_sec()))
    outtxt.write(str.format("{0:.9f} ", vr[0]))
    outtxt.write(str.format("{0:.9f} ", vr[1]))
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0 ')
    outtxt.write('0\n')


if __name__ == '__main__':

    outtxt = open('/home/jitete/drones/src/correlation_flow/script/cf.txt','w')
    outtxt.write('# text file' + '\n# format: time stamp x y z qx qy qz qw\n')

    rospy.init_node('bodyToNED')
    rospy.Subscriber('/imu/data', Imu, imucallback)
    pub = rospy.Publisher('velocity', TwistStamped, queue_size=0)	
    rospy.Subscriber('/correlation_flow_node/corrFlow_velocity', TwistStamped, toNED)
    rospy.spin()


