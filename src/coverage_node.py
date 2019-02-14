#!/usr/bin/python
#
# coverage_node.py
#
#  Created on: today
#      Author: Moshe Samson
#

import sys
import numpy as np
import os

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from turtlesim.msg import Pose
import globals

from Coverage import *

# positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
positions = []


def pose_callback(pose_msg):
    # Keep track of robot position:
    print "+++position callback+++"
    globals.position_file.write(pose_msg.position + "\n")

    return


def move_blindly(last_d, new_d, distance):
    print("(%s) Moving %s ..." % (globals.robot_name, new_d)),

    rotation_angle_deg = 0
    if (last_d == 'W' and new_d == 'N') or (last_d == 'N' and new_d == 'E') or \
            (last_d == 'E' and new_d == 'S') or (last_d == 'S' and new_d == 'W'):
        rotation_angle_deg = 90
    elif (last_d == 'W' and new_d == 'S') or (last_d == 'S' and new_d == 'E') or \
            (last_d == 'E' and new_d == 'N') or (last_d == 'N' and new_d == 'W'):
        rotation_angle_deg = -90
    else:
        rotation_angle_deg = 0

    angle = np.deg2rad(rotation_angle_deg)
    angle_vel = angle * 0.5

    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.5  # speed
    rotate_msg = Twist()
    rotate_msg.angular.z = angle_vel
    stay_put_msg = Twist()

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0
    while current_angle < angle:
        globals.pub.publish(rotate_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angle_vel * (t1 - t0)
    globals.pub.publish(stay_put_msg)

    current_distance = 0.0
    t0 = rospy.Time.now().to_sec()
    while current_distance < distance:
        globals.pub.publish(move_forward_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = 0.5 * (t1 - t0)

    globals.pub.publish(stay_put_msg)

    print "Done."


def open_position_file():
    global positions_file



def main():
    rospy.init_node('coverage_node', argv=sys.argv)
    starting_location = (0, 0)
    globals.robot_size = 0.35  # meters!
    globals.robot_name = ''

    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        rospy.loginfo("has robot size!")

    if rospy.has_param('~robot_name'):
        globals.robot_name = rospy.get_param('~robot_name')
        print "robot_name: {0}".format(globals.robot_name)

    open_position_file()

    # Publisher - context is inside namespace
    globals.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

    # print_topics()

    # Listener for pose
    globals.sub = rospy.Subscriber(globals.robot_name + "/pose", Pose, pose_callback)

    # initiate map service
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a width %d X height %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))
        rospy.loginfo("position: " + str(response.map.info.origin.position))

        create_occupancy_grid_using_robot_size(response.map, globals.robot_size)

        # find robot's position
        globals.tf_listener = tf.TransformListener()
        globals.tf_listener.waitForTransform('/map', globals.robot_name + "/base_footprint", rospy.Time(0), rospy.Duration(10))

        # # compute robot initial position in grid
        # starting_location_grid = (int(math.floor(
        #     (starting_location[1] - response.map.info.origin.position.y) / robot_size)),
        #                           int(math.floor(
        #                               (starting_location[0] - response.map.info.origin.position.x) / robot_size))
        # )

        starting_location_grid = (len(globals.grid)/2, len(globals.grid[0])/2)
        print "starting location grid: " + str(starting_location_grid)

        # switch to coarse grid cell, each cell of size 4D
        switch_to_coarse_grid()

        # find free initial location in the coarse grid
        starting_location_coarse_grid = (starting_location_grid[0] / 2, starting_location_grid[1] / 2)
        print "starting location coarse grid: " + str(starting_location_coarse_grid)
        if globals.coarse_grid[starting_location_coarse_grid[0]][starting_location_coarse_grid[1]]:
            rospy.loginfo("Original Starting location is occupied! Finding the closest free starting location...")

            # perform bfs over all edges, then get the first one which isn't occupied
            starting_location_coarse_grid = get_free_starting_location(globals.coarse_grid, starting_location_coarse_grid)
            rospy.loginfo("starting location coarse grid: " + str(starting_location_coarse_grid))

        # Find all the free cells that are reachable from the robot's initial position and ignore all the other cells.
        reachable_cells_coarse_grid = get_reachable_cells(globals.coarse_grid, starting_location_coarse_grid)

        # Make sure all unreachable cells considered as occupied.
        for row in xrange(len(globals.coarse_grid)):
            for col in xrange(len(globals.coarse_grid[0])):
                if (row, col) not in reachable_cells_coarse_grid:
                    globals.coarse_grid[row][col] = True

        # create spanning tree of robot
        coarse_grid_edges = get_edges_from_grid(globals.coarse_grid)
        coarse_grid_graph = create_graph(coarse_grid_edges)
        coarse_grid_mst = mst(starting_location_coarse_grid, coarse_grid_graph)

        # get coverage path in the fine grid. using the mst of the coarse grid
        path = get_coverage_path_from_mst(coarse_grid_mst, starting_location_grid)

        # use send_goals code, create mini-plan for each move, then moving the robot through that plan
        last_p = path[0]
        if last_p.GoUp() == path[1]:
            last_d = 'N'
        elif last_p.GoRight() == path[1]:
            last_d = 'E'
        elif last_p.GoDown() == path[1]:
            last_d = 'S'
        else:
            last_d = 'W'

        begin_time = rospy.Time.now()
        covered_cells_count = 0


        # move the robot along the coverage path
        for p in path:
            # Check if going along the same path as before, and stop only when changing direction, then call send_goals
            # if (last_d == 'N' and last_p.GoUp() == p) or \
            #         (last_d == 'E' and last_p.GoRight() == p) or \
            #         (last_d == 'S' and last_p.GoDown() == p) or \
            #         (last_d == 'W' and last_p.GoLeft() == p):
            #     last_p = p
            #     covered_cells_count += 1
            #     print "continue..."
            #     continue
            # else:
                new_d = ""
                if last_p.GoUp() == p:
                    new_d = 'N'
                elif last_p.GoRight() == p:
                    new_d = 'E'
                elif last_p.GoDown() == p:
                    new_d = 'S'
                else:
                    new_d = 'W'

                # print "%s -- X: %d, Y: %d" % (robot_name, int(last_p.col * robot_size + response.map.info.origin.position.x),
                #                                 int(last_p.row * robot_size + response.map.info.origin.position.y))

                move_blindly(last_d, new_d, globals.robot_size)
                print_location()
                last_d = new_d

                # succeeded = send_goals(
                #     int(last_p.col * robot_size + response.map.info.origin.position.x),
                #     int(last_p.row * robot_size + response.map.info.origin.position.y))

                last_p = p

        finish_time = rospy.Time.now()
        positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
        positions_file.writelines(positions)
        positions_file.close()
        #
        # # print statistics of this problem
        # with open("/home/moshe/catkin_ws/src/coverage_4/src/coverage_stat.txt", "w+") as stats_file:
        #     print "statrting to write coverage stats!"
        #     stats_file.write("The number of cells accessible from the robot initial position: " +
        #                      str(len(reachable_cells_coarse_grid)) + '\n')
        #     # stats_file.write("The robots initial location in the 4D grid: " + str(starting_location_coarse_grid))
        #     stats_file.write("# Covered cells: " + str(covered_cells_count) + '\n')
        #     stats_file.write("Total Coverage Time: " + str((finish_time.to_sec() - begin_time.to_sec())) + '\n')
        #     rospy.loginfo("finished writing coverage_stat.txt")

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def print_location():
    global positions_file
    try:
        (trans, rot) = globals.tf_listener.lookupTransform("/map",
                                                           globals.robot_name + "/base_footprint",
                                                           rospy.Time(0))
        location = (trans[0], trans[1])

        positions.append("(%s) : %f,%f \n" % (globals.robot_name, location[0], location[1]))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        rospy.logerr("Service call failed: %s" % e)


def print_topics():
    for k, v in rospy.get_published_topics():
        print k + "  -->  " + v


if __name__ == "__main__":
    globals.__init__()
    main()
