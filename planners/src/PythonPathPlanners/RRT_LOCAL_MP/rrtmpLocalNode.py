#!/usr/bin/env python

import CollisionTest as ct
import ExtractPath as ep
import NearestNeighbour as nn
import Sampling as sp
import MotionPrimitiveGeneration as mpg
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
import numpy as np
from numpy import linalg as la

rRadius = rospy.get_param('/rrtmpNode/robot_radius', 5)
gTolerance = rospy.get_param('/rrtmpNode/goal_tolerance', 10)
obs_inflation_radius = rospy.get_param('/rrtmpNode/obstacle_inflation_radius', 1)
step_size = rospy.get_param('/rrtmpNode/step_size', 10)
angle_threshold = rospy.get_param('/rrtmpNode/angle_threshold', 0.785)
mpArray = rospy.get_param('/rrtmpNode/motion_primitive_array', [[0.5, -0.2], [0.5, 0.0], [0.5, 0.2]])
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

def map_callback(data):
    global obs
    global map_width
    global map_height
    global map_resolution
    global maxX
    global minX
    global maxY
    global minY
    global mpArray

    map_resolution = data.info.resolution
    map_width = data.info.width
    map_height = data.info.height
    global_map = data.data

    for i in range(len(mpArray)):
        mpArray[i][0] = mpArray[i][0]/map_resolution

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
             2*np.arcsin(data.pose.orientation.z)]
    rospy.loginfo("Goal Pose: (%d, %d, %f)", qGoal[0], qGoal[1], qGoal[2])


def init_callback(data):
    global qInit
    global init_set
    init_set = True
    qInit = [data.pose.pose.position.x/map_resolution, data.pose.pose.position.y/map_resolution,
             2*np.arcsin(data.pose.pose.orientation.z)]
    rospy.loginfo("Initial Pose: (%d, %d, %f)", qInit[0], qInit[1], qInit[2])


def rrtmp_ros():
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
    global mpArray
    ########################################
    rospy.init_node('rrtmp_ros', anonymous=True)
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
            tree.append(rc.Vertex(cntId, qInit, [], [0, 0]))
            qGoal_p = qGoal
            qInit_p = qInit
            points.poses = []

        if goalFound < 0 and goal_set is True and init_set is True:

            qRand = rc.Vertex([], sp.sampling(maxX, maxY, minX, minY), [], [])
            if ct.checkCollision(qRand, obs, rRadius) > 0:
                continue

            qNear = nn.nearestNeighbour(qRand, tree)

            branch = mpg.generateMP(qNear, cntId, 3, 0.25, mpArray, obs, rRadius)

            if len(branch) > 0:
                cntId = branch[-1].id
            else:
                continue

            tree.extend(branch)

            for i in range(len(branch)):
                point = Pose()
                point.position = Point(branch[i].pose[0]*map_resolution, branch[i].pose[1]*map_resolution, 0)
                quat = [0, 0, np.sin(branch[i].pose[2]/2), np.cos(branch[i].pose[2]/2)]
                point.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                points.poses.append(point)
            tree_pub.publish(points)

            for i in range(len(branch)):
                if la.norm(np.subtract(branch[i].pose[0:2], qGoal[0:2])) < gTolerance:
                    goalFound = 1
                    goal_set = False
                    p = ep.extractPath(branch[i], tree, qInit)
                    if p == 0:
                        rospy.loginfo("Couldn't Find Path, Error in Graph Search")
                        break
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
                        break


if __name__ == '__main__':
    try:
        rrtmp_ros()
    except rospy.ROSInterruptException:
        pass
