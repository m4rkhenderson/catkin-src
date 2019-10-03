#!/usr/bin/env python

import CollisionTest as ct
import ExtractPath as ep
import NearestNeighbour as nn
import NearVertices as nv
import Sampling as sp
import Steering as st
import RewireTree as rt
import RRTClasses as rc
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import *
import numpy as np
from numpy import linalg as la

rRadius = 5
gTolerance = 10
obs_inflation_radius = 1
obs = rc.Obstacle([], obs_inflation_radius)
maxX = 100
minX = 0
maxY = 100
minY = 0
map_width = 0
map_height = 0
map_resolution = 0.005
qGoal = [0, 0]
qGoal_p = [0, 0]
qInit = [0, 0]
qInit_p = [0, 0]
goal_set = False
init_set = False
goalFound = -1
tree = []
cntId = 0
step_size = 10
angle_threshold = 0.785


def map_callback(data):
    global obs
    global map_width
    global map_height
    global map_resolution
    global maxX
    global minX
    global maxY
    global minY

    map_resolution = data.info.resolution
    map_width = data.info.width
    map_height = data.info.height
    global_map = data.data

    maxX = 0
    minX = map_width
    maxY = 0
    minY = map_height

    for i in range(len(global_map)):
        if global_map[i] > 0:

            x = i % map_width
            y = int(i/map_width)
            obs.points.append([x, y])

            if x > maxX:
                maxX = x
            if x < minX:
                minX = x
            if y > maxY:
                maxY = y
            if y < minY:
                minY = y

    rospy.loginfo("Obstacle Number: %d", len(obs.points))


def goal_callback(data):
    global qGoal
    global goal_set
    goal_set = True
    qGoal = [data.pose.position.x/map_resolution, data.pose.position.y/map_resolution,
             data.pose.orientation.z]
    rospy.loginfo("Goal Pose: (%d,%d, %f)", qGoal[0], qGoal[1], qGoal[2])


def init_callback(data):
    global qInit
    global init_set
    init_set = True
    qInit = [data.pose.pose.position.x/map_resolution, data.pose.pose.position.y/map_resolution,
             data.pose.pose.orientation.z]
    rospy.loginfo("Initial Pose: (%d,%d, %f)", qInit[0], qInit[1], qInit[2])


def rrt_ros():
    # global variables necessary for planner
    global rRadius
    global obs
    global maxX
    global minX
    global maxY
    global minY
    global map_width
    global map_height
    global qGoal
    global qGoal_p
    global qInit
    global qInit_p
    global goal_set
    global init_set
    global goalFound
    global tree
    global cntId
    global step_size
    ########################################
    rospy.init_node('prm_ros', anonymous=True)
    rate = rospy.Rate(30)
    path_pub = rospy.Publisher('global_path', Path, queue_size=100)
    tree_pub = rospy.Publisher('tree', PoseArray, queue_size=100)
    rospy.Subscriber('map', OccupancyGrid, map_callback)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, init_callback)

    points = PoseArray()
    points.header.frame_id = "tree"

    while not rospy.is_shutdown():
        rate.sleep()
        # do path planning here
        if qGoal_p != qGoal or qInit_p != qInit:
            tree = []
            cntId = 0
            goalFound = -1
            tree.append(rc.Vertex(cntId, qInit, [], 0))
            qGoal_p = qGoal
            qInit_p = qInit
            points.poses = []

        if goalFound < 0 and goal_set is True and init_set is True:

            qRand = rc.Vertex([], sp.sampling(maxX, maxY, minX, minY), [], 0)
            if ct.checkCollision(qRand, obs, rRadius) > 0:
                rospy.loginfo("Invalid Random Configuration")
                continue

            qNear = nn.nearestNeighbour(qRand, tree)

            qNew = st.steering(qRand, qNear, cntId, step_size, angle_threshold)

            if ct.checkCollision(qNew, obs, rRadius) > 0:
                rospy.loginfo("Invalid New Configuration")
                continue

            cntId = cntId + 1
            qNew.id = cntId
            qNew.pid = [qNear.id]

            volume = (maxX - minX)*(maxY-minY)
            tree = rt.rewire(qNew, tree, volume, step_size)

            rospy.loginfo("Added New Connections to tree %d: (%d, %d, %f)",
                          cntId, qNew.pose[0], qNew.pose[1], qNew.pose[2])

            point = Pose()
            point.position = Point(qNew.pose[0]*map_resolution, qNew.pose[1]*map_resolution, 0)
            quat = [0, 0, np.sin(qNew.pose[2]/2), np.cos(qNew.pose[2]/2)]
            point.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
            points.poses.append(point)
            tree_pub.publish(points)

            if la.norm(np.subtract(qNew.pose[0:2], qGoal[0:2])) < gTolerance:
                goalFound = 1
                goal_set = False
                p = ep.extractPath(tree[-1], tree, qInit)
                if p == 0:
                    rospy.loginfo("Couldn't Find Path, Error in Graph Search")
                else:
                    rospy.loginfo("Found Path!")
                    path = Path()
                    for i in range(len(p[0])):
                        pose = PoseStamped()
                        pose.pose.position = Point(p[0][i]*map_resolution, p[1][i]*map_resolution, 0)
                        quat = [0, 0, np.sin(p[2][i]/2), np.cos(p[2][i]/2)]
                        pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                        path.header.frame_id = "global_path"
                        path.poses.append(pose)
                        rospy.loginfo("Path Point %d: (%d, %d, %f)", i, p[0][i], p[1][i], p[2][i])
                    path_pub.publish(path)  # then publish


if __name__ == '__main__':
    try:
        rrt_ros()
    except rospy.ROSInterruptException:
        pass