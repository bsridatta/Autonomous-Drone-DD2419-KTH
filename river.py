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
import rospy
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
idx = 0
move = 0
wait = 1
stop_threshold =1
dist_threshold = 0.5
NPOINTS = 12
callback_data = None
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
    global callback_data
    global odom
    global idx
    global rx1,ry1,yaw_degrees
    callback_data = msg

    if pose_callback.state == move:
        # Compute distance between current pose and goal
        #storing in 2 variablestf_buf
        pos = callback_data.pose.position
        orient = callback_data.pose.orientation
        #print("callback position",pos)
        print("idx",idx)
        #print("please go inside")
        #print(len(rx1))
        if len(rx1)>0:
            #print("going_inside")
            onepointpublish(rx1[idx],ry1[idx],yaw_degrees[idx])
            local_map_to_odom = map_to_odom(rx1[idx],ry1[idx])
            #print("local map to odom",local_map_to_odom)
            d =np.array([0,0])
            d = np.array([local_map_to_odom.x-pos.x,local_map_to_odom.y-pos.y])
            print("d",d)
            print("pos.x",pos.x)
            distance = np.linalg.norm(d)
        #publish_cmd(idx)
            print("distance",distance)
            if idx == 4:
                distance = distance -0.70
            if idx==5:
               distance = distance - 0.4
            if idx==6:
                #dist_threshold=0.72
                #distance = distance - 0.1
                for i in range(40000000):
                    if i > 39999988:
                        print(i)
                for i in range(45):
                    d = distance -(i*0.01)

                    #print(distance)
                    if d < dist_threshold:
                        print(d)
                        distance = d
                        break
            if idx==7:
                #2.5
                #dist_threshold=0.72
                for i in range(5000000):
                    if i > 4999988:
                        print(i)
                for i in range(70):
                    d = distance -(i*0.01)

                    #print(distance)
                    if d < dist_threshold:
                        distance = d
                        break
            if idx==8:
                #2.5
                #dist_threshold=0.72
                for i in range(5000000):
                    if i > 4999988:
                        print(i)
                for i in range(150):
                    d = distance -(i*0.01)

                    #print(distance)
                    if d < dist_threshold:
                        distance = d
                        break

            if idx==9:

                  #dist_threshold=0.72
                  for i in range(2000000):
                    if i > 1999988:
                        print(i)
                  for i in range(100):
                      d = distance -(i*0.01)

                      #print(distance)
                      if d < dist_threshold:
                          distance = d
                          break


            if(distance < dist_threshold):
                d =np.array([0,0])
                distance = 0.0
                print("reached at index",idx)
                pose_callback.state = wait
            # else:
            #     print("let's wait")
            #     pose_callback.state = wait

    elif pose_callback.state == wait:
        # Wait here for some iterations, counter here which will keep counting timpes
        pose_callback.count = pose_callback.count + 1
        #publish_cmd(idx)
        #onepointpublish(rx1[idx],ry1[idx])
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
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.4
    # goal.pose.orientation.x = 0.0
    # goal.pose.orientation.y = 0.0
    # goal.pose.orientation.z = -0.999021480034635
    # goal.pose.orientation.w = -0.0442276206618389
    # roll, pitch, yaw = euler_from_quaternion((goal.pose.orientation.x,
    #                                           goal.pose.orientation.y,
    #                                           goal.pose.orientation.z,
    #                                           goal.pose.orientation.w))
    #print(goal.header.stamp)
    if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')
    if goal_odom:
        #print(goal_odom.header.stamp)
        cmd = Position()

        cmd.header.stamp = rospy.Time.now()
        #print(goal_odom.header.frame_id) odom
        cmd.x = goal_odom.pose.position.x
        cmd.y = goal_odom.pose.position.y
        cmd.z = goal_odom.pose.position.z
        #print(cmd.x,cmd.y,cmd.z)
        roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))


        #cmd.yaw = math.degrees(yaw)
        cmd.yaw = yaw1
        odom = cmd


    #print "DONE"



def map_to_odom(x,y):

    #rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.4

    #goal.pose.orientation.z = 0
    # if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
    #     rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
    #     return
    if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(3)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    #trans = tf_buf.lookup_transform('cf1/odom',"map",rospy.Time(0),rospy.Duration(0.4))
    goal_odom = tf_buf.transform(goal, 'cf1/odom')
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    #print(trans.header.frame_id)
    cmd.x = goal_odom.pose.position.x
    cmd.y = goal_odom.pose.position.y
    cmd.z = goal_odom.pose.position.z
    #print(cmd.x,cmd.y,cmd.z)
    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)


    map_to_odom = cmd
    return map_to_odom








pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)


pose_callback.state = move
pose_callback.count = 0

def main():
    global rx1
    global ry1
    global yaw_degrees
    # rx1 =           [0.5, 2.0, 2.0, 2.0,2.0, 0.0,    0.0, 0.0]
    # ry1=            [0.0, 0.0, 0.0, 2.0,2.0, 2.0,   2.0, 0.0]
    # yaw_degrees =   [0.0, 0.0,90.0,90.0,180.0,180.0,270.0,270.0]
    rx1 =             [0.5, 2.0, 2.0, 2.0, 2.0, 2.0, 1.5,    0.5,  0.0,   0.0,   0.0,  0.0]
    ry1 =             [0.0, 0.0, 0.0, 0.5, 1.5, 1.5,  2.0,   2.0,  1.5,   1.0,   0.0,  0.0]
    yaw_degrees =     [0.0, 0.0, 90.0,90.0,90.0, 180.0, 180.0, 180.0, 270.0, 270.0, 270.0,360.0]

    print(rx1)
    #while rospy

    # onepointpublish(rx1[idx],ry1[idx])
    # onepointpublish(rx1[idx],ry1[idx])
    # onepointpublish(rx1[idx],ry1[idx])


if __name__ == '__main__':
    main()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if odom:
            #print("yahio")

            #print(odom.header.frame_id)
            #print(odom)
            pub_cmd.publish(odom)
        rate.sleep()
