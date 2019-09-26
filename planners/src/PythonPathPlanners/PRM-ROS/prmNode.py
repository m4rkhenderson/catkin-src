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

obs_inflation_radius = 1
obs = pc.Obstacle([], 1)


def map_callback(data):
    global obs
    map_width = data.MapMetaData.width
    j = 0
    global_map = data.data
    for i in range(len(global_map)):
        if global_map[i] > 0:
            obs.points.append([j, i])
        j = j + 1
        if j > map_width:
            j = 0


def prm_ros():
    rospy.init_node('prm_ros', anonymous=True)
    rate = rospy.Rate(10)
    path_pub = rospy.Publisher('global_path', Path, queue_size=100)
    rospy.Subscriber('map', OccupancyGrid, map_callback)

    while not rospy.is_shutdown():
        # do path planning here
        path_pub.publish()  # then publish
        rate.sleep()


if __name__ == '__main__':
    try:
        prm_ros()
    except rospy.ROSInterruptException:
        pass
