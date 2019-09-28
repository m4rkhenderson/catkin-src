#!/usr/bin/env python

import CollisionTest as ct
import ExtractPath as ep
import NearestNeighbour as nn
import NearVertices as nv
import Sampling as sp
import Steering as st
import PRMClasses as pc
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
import numpy as np
from numpy import linalg as la

rRadius = 10
gTolerance = 10
obs_inflation_radius = 1
obs = pc.Obstacle([], obs_inflation_radius)
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
    qGoal = [data.pose.position.x/map_resolution, data.pose.position.y/map_resolution]
    rospy.loginfo("Goal Pose: (%d,%d)", qGoal[0], qGoal[1])


def init_callback(data):
    global qInit
    global init_set
    init_set = True
    qInit = [data.pose.pose.position.x/map_resolution, data.pose.pose.position.y/map_resolution]
    rospy.loginfo("Initial Pose: (%d,%d)", qInit[0], qInit[1])


def prm_ros():
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
    rate = rospy.Rate(60)
    path_pub = rospy.Publisher('global_path', Path, queue_size=100)
    roadmap_pub = rospy.Publisher('roadmap', PoseArray, queue_size=100)
    rospy.Subscriber('map', OccupancyGrid, map_callback)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, init_callback)

    points = PoseArray()
    points.header.frame_id = "roadmap"

    while not rospy.is_shutdown():
        rate.sleep()
        # do path planning here
        if qGoal_p != qGoal or qInit_p != qInit:
            tree = []
            cntId = 0
            goalFound = -1
            tree.append(pc.Vertex(cntId, qInit, []))
            qGoal_p = qGoal
            qInit_p = qInit
            points.poses = []

        if goalFound < 0 and goal_set is True and init_set is True:

            qRand = pc.Vertex([], sp.sampling(maxX, maxY, minX, minY), [])
            if ct.checkCollision(qRand, obs, rRadius) > 0:
                rospy.loginfo("Invalid Random Configuration")
                continue

            qNear = nn.nearestNeighbour(qRand, tree)

            qNew = st.steering(qRand, qNear, cntId, step_size)

            if ct.checkCollision(qNew, obs, rRadius) > 0:
                rospy.loginfo("Invalid New Configuration")
                continue

            cntId = cntId + 1
            qNew.id = cntId

            vNear = nv.nearVertices(qNew, tree, step_size)
            if vNear is 0:
                vNear.append(qNear.id)
            qNew.pid = vNear

            tree.append(qNew)
            rospy.loginfo("Added New Connections to Roadmap: %d, (%d,%d)", cntId, qNew.pose[0], qNew.pose[1])

            pose = Pose()
            pose.position = Point(qNew.pose[0]*map_resolution, qNew.pose[1]*map_resolution, 0)

            points.poses.append(pose)
            roadmap_pub.publish(points)

            if la.norm(np.subtract(qNew.pose, qGoal)) < gTolerance:
                goalFound = 1
                goal_set = False
                p = ep.extractPath(qNew, tree, qInit)
                if p == 0:
                    rospy.loginfo("Couldn't Find Path, Error in Graph Search")
                else:
                    rospy.loginfo("Found Path!")
                    path = Path()
                    for i in range(len(p[0])):
                        pose = PoseStamped()
                        pose.pose.position = Point(p[0][i]*map_resolution, p[1][i]*map_resolution, 0)
                        path.header.frame_id = "global_path"
                        path.poses.append(pose)
                        rospy.loginfo("Path Point %d: (%d,%d)", i, p[0][i], p[1][i])
                    path_pub.publish(path)  # then publish


if __name__ == '__main__':
    try:
        prm_ros()
    except rospy.ROSInterruptException:
        pass
