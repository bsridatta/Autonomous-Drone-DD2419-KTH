#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.pyplot as plt
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import numpy as np
import tf
idx = 0
move = 0
wait = 5
stop_threshold =10
dist_threshold = 0.2
NPOINTS = 12
odom = None

 # is of opsition type
rx1=[]
ry1=[]
yaw_degrees=[]
rospy.init_node('planner')
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def pose_callback(msg):
#indexing every position
# the msg will be of type Posestamped
    global odom
    global idx
    global rx1,ry1,yaw_degrees
    callback_data = msg
    global stamp
    if pose_callback.state == move:
        # Compute distance between current pose and goal
        #storing in 2 variablestf_buf
        pos = callback_data.pose.position
        orient = callback_data.pose.orientation
        stamp = callback_data.header.stamp
        if len(rx1)>0:
            goal_map_to_odom = onepointpublish(rx1[idx],ry1[idx],yaw_degrees[idx])
            d =np.array([0,0])
            d = np.array([goal_map_to_odom.x-pos.x,goal_map_to_odom.y-pos.y])
            print("d",d)
            # print("pos.x",pos.x)
            distance = np.linalg.norm(d)
            print("distance",distance)
            print("idx",idx)

            if(distance < dist_threshold):
                d =np.array([0,0])
                distance = 0.0
                print("reached at index",idx)
                pose_callback.state = wait

    elif pose_callback.state == wait:
        # Wait here for some iterations, counter here which will keep counting timpes
        pose_callback.count = pose_callback.count + 1
        if (pose_callback.count > stop_threshold):
            #We've waited enough, start moving again to next point
            print("next goal")
            pose_callback.state = move
            pose_callback.count = 0

            idx = idx + 1
            print("idx inside nextgoal",idx)
            idx = idx % NPOINTS





def onepointpublish(x,y,yaw1):
    global odom
    rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = stamp
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.4
    if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')
    if goal_odom:
        #print(goal_odom.header.stamp)
        cmd = Position()

        cmd.header.stamp = rospy.Time.now()
        cmd.x = goal_odom.pose.position.x
        cmd.y = goal_odom.pose.position.y
        cmd.z = goal_odom.pose.position.z
        cmd.yaw = yaw1
        odom = cmd
        return odom



pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)


pose_callback.state = move
pose_callback.count = 0

def main():
    global rx1
    global ry1
    global yaw_degrees

    rx1 =             [0.5, 2.0, 2.0, 2.0, 2.0, 2.0, 1.5,    0.5,  0.0,   0.0,   0.0,  0.0]
    ry1 =             [0.0, 0.0, 0.0, 0.5, 1.5, 1.5,  2.0,   2.0,  1.5,   0.5,   0.0,  0.0]
    yaw_degrees =     [0.0, 0.0, 90.0,90.0,90.0, 180.0, 180.0, 180.0, 270.0, 270.0, 270.0,360.0]

    print(rx1)
    #while rospy


if __name__ == '__main__':
    main()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        if odom:
            pub_cmd.publish(odom)
        rate.sleep()
