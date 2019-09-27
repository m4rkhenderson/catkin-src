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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from numpy import linalg as la

rRadius = 10
gTolerance = 10
obs_inflation_radius = 1
obs = pc.Obstacle([], 1)
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
    map_resolution = data. MapMetaData.resolution
    map_width = data.MapMetaData.width
    map_height = data.MapMetaData.height
    j = 0
    global_map = data.data
    for i in range(len(global_map)):
        if global_map[i] > 0:
            obs.points.append([j, i])
        j = j + 1
        if j > map_width:
            j = 0


def goal_callback(data):
    global qGoal
    global goal_set
    goal_set = True
    qGoal = [data.pose.position.x, data.pose.position.y]


def init_callback(data):
    global qInit
    global init_set
    init_set = True
    qInit = [data.pose.pose.position.x, data.pose.pose.position.y]


def prm_ros():
    # global variables necessary for planner
    global rRadius
    global obs_inflation_radius
    global obs
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
    rate = rospy.Rate(10)
    path_pub = rospy.Publisher('global_path', Path, queue_size=100)
    rospy.Subscriber('map', OccupancyGrid, map_callback)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, init_callback)

    while not rospy.is_shutdown():
        rate.sleep()
        # do path planning here
        if qGoal_p != qGoal or qInit_p != qInit:
            tree = []
            cntId = 0
            goalFound = -1
            tree.append(pc.Vertex(cntId, qInit, []))
        if goalFound < 0 and qGoal != qInit:

            qRand = pc.Vertex([], sp.sampling(map_width, map_height), [])
            if ct.checkCollision(qRand, obs, rRadius) > 0:
                continue

            qNear = pc.Vertex([], nn.nearestNeighbour(qRand, tree), [])
            qNew = st.steering(qRand, qNear, cntId, step_size)
            if ct.checkCollision(qNew, obs, rRadius) > 0:
                continue
            cntId = cntId + 1

            vNear = nv.nearVertices(qNew, tree, step_size)
            qNew.pid = vNear
            tree.append(qNew)

            if la.norm(np.subtract(qNew.pose, qGoal)) < gTolerance:
                goalFound = 1
                p = ep.extractPath(qNew, tree, qInit)
                if p == 0:
                    rospy.loginfo("Couldn't Find Path, Error in Graph Search")
                else:
                    path = Path()
                    for i in range(len(p)):
                        p[i][0] = p[i][0]*map_resolution
                        p[i][1] = p[i][1]*map_resolution
                        pose = PoseStamped()
                        pose.pose.position.x = p[i][0]
                        pose.pose.position.y = p[i][0]
                        path.poses.append(pose)
                    path_pub.publish(path)  # then publish


if __name__ == '__main__':
    try:
        prm_ros()
    except rospy.ROSInterruptException:
        pass
