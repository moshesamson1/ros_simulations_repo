#!/usr/bin/python
#
# coverage_node.py
#
#  Created on: today
#      Author: Moshe Samson
#

import sys

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from turtlesim.msg import Pose

from Coverage import *
from Entities import Direction, deprecated

# positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
positions = []


def pose_callback(pose_msg):
    # Keep track of robot position:
    print "+++position callback+++"
    globals.position_file.write(pose_msg.position + "\n")

    return


def move(last_d, new_d, distance=0.0, percentage=0.0, use_map=True):
    if not use_map:
        move_blindly(last_d, new_d, distance, percentage)
    else:
        move_smartly(last_d, new_d, distance, percentage)


def move_smartly(last_d, new_d, distance, percentage):
    print "(%s) Moving %s -> %s ..." % (globals.robot_name, last_d, new_d),

    # todo: use vectors to represent direction and compute angles
    if (last_d == Direction.W and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.E) or \
            (last_d == Direction.E and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.W):
        rotation_angle_deg = 90
    elif (last_d == Direction.W and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.E) or \
            (last_d == Direction.E and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.W):
        rotation_angle_deg = -90
    elif last_d == Direction.Z or last_d == new_d:
        rotation_angle_deg = 0
    else:
        rospy.logerr("Wrong directions. trying to switch from %s to %s" % (last_d, new_d))
        exit(-1)

    angle = np.deg2rad(rotation_angle_deg)

    if new_d == Direction.N:
        direction = Direction.N
    elif new_d == Direction.E:
        direction = Direction.E
    elif new_d == Direction.S:
        direction = Direction.S
    elif new_d == Direction.W:
        direction = Direction.W

    turn_toward(get_euler_orientation()[2] + angle)
    move_toward((10.0, 10.0), direction)  # check!

    print "Done. %f" % percentage


@deprecated
def move_blindly(last_d, new_d, distance, percentage):
    print "(%s) Moving %s -> %s ..." % (globals.robot_name, last_d, new_d),

    # todo: use vectors to represent direction and compute angles
    if (last_d == Direction.W and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.E) or \
            (last_d == Direction.E and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.W):
        rotation_angle_deg = 90
    elif (last_d == Direction.W and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.E) or \
            (last_d == Direction.E and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.W):
        rotation_angle_deg = -90
    elif last_d == Direction.Z or last_d == new_d:
        rotation_angle_deg = 0
    else:
        rospy.logerr("Wrong directions. trying to switch from %s to %s" % (last_d, new_d))
        exit(-1)

    angle = np.deg2rad(rotation_angle_deg)
    angle_vel = angle * 0.5  # angular speed

    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.5  # directional speed
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

    print "Done. %f" % percentage


def main():
    rospy.init_node('coverage_node', argv=sys.argv)
    globals.robot_size = 0.35  # meters!
    globals.robot_name = ''

    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        globals.robot_size = float(rospy.get_param('~robot_size'))

    if rospy.has_param('~robot_name'):
        globals.robot_name = rospy.get_param('~robot_name')

    if rospy.has_param('~init_pos'):
        str_pos = rospy.get_param('~init_pos').split()
        globals.init_pos = float(str_pos[0]), float(str_pos[1])

        # Publisher - context is inside namespace
    globals.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

    # Listener for pose
    globals.sub = rospy.Subscriber(globals.robot_name + "/pose", Pose, pose_callback)

    # initiate map service
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        globals.response = get_static_map()
        rospy.loginfo("Received a width %d X height %d map @ %.3f m/px" % (
            globals.response.map.info.width, globals.response.map.info.height, globals.response.map.info.resolution))
        rospy.loginfo("position: " + str(globals.response.map.info.origin.position))

        create_occupancy_grid_using_robot_size(globals.response.map, globals.robot_size)
        print "(%s) before tf" % globals.robot_name
        # find robot's position
        globals.tf_listener = tf.TransformListener()
        globals.tf_listener.waitForTransform('/map', globals.robot_name + "/base_footprint", rospy.Time(0),
                                             rospy.Duration(10))
        print "(%s) after tf" % globals.robot_name

        # point robot to the correct direction
        set_orientation(0.0)

        # compute robot initial position in grid
        starting_location = get_location()
        starting_location_grid = \
            (int(math.floor((starting_location[1] - globals.response.map.info.origin.position.y) / globals.robot_size)),
             int(math.floor((starting_location[0] - globals.response.map.info.origin.position.x) / globals.robot_size)))

        print "(%s) init pos: %s" % (globals.robot_name, str(globals.init_pos))
        print "(%s) starting location: %s" % (globals.robot_name, str(starting_location))
        print "(%s) starting location grid: %s" % (globals.robot_name, str(starting_location_grid))

        # switch to coarse grid cell, each cell of size 4D
        switch_to_coarse_grid()

        # find free initial location in the coarse grid
        starting_location_coarse_grid = int((starting_location_grid[0] - 1) / 2.0), int(
            (starting_location_grid[1] - 1) / 2.0)
        print "(%s) starting_location_coarse_grid: %s" % (globals.robot_name, str(starting_location_coarse_grid))
        print "(%s) coarse grid size: %f,%f" % (
        globals.robot_name, len(globals.coarse_grid), len(globals.coarse_grid[0]))

        coarse_grid_edges = get_edges_from_grid(globals.coarse_grid)
        coarse_grid_graph = create_graph(coarse_grid_edges)
        coarse_grid_mst = mst(starting_location_coarse_grid, coarse_grid_graph)

        # get coverage path in the fine grid. using the mst of the coarse grid
        path = get_coverage_path_from_mst(coarse_grid_mst, starting_location_grid)

        # ~~~
        last_p = path[0]
        last_d = Direction.Z

        begin_time = rospy.Time.now()
        # move the robot along the coverage path
        # ignore first step of the path, as it is the starting position

        for p_ind in xrange(1, len(path)):
            p = path[p_ind]

            if last_p.GoUp() == p:
                new_d = Direction.N
            elif last_p.GoRight() == p:
                new_d = Direction.E
            elif last_p.GoDown() == p:
                new_d = Direction.S
            else:
                new_d = Direction.W

            percentage = float(p_ind) / float(len(path))
            move(last_d, new_d, percentage=percentage, use_map=True)
            # print_location()
            last_d, last_p = new_d, p

        finish_time = rospy.Time.now()

        positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
        positions_file.writelines(positions)
        positions_file.close()

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def set_orientation(target_orientation_z):
    print "*** set_orientation ***"
    turn_toward(target_orientation_z)


def move_toward(target_position, direction, eps=0.1):
    # type: (tuple, Direction, float) -> None
    """
    moving from current location to specific location, in a straight line
    :param target_position:
    :param direction:
    :param eps:
    :return:
    """

    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.5

    stay_put_msg = Twist()

    current_location = get_location()

    # compute index to compare against. If moving south or north, compare rows. Otherwise compare columns
    index = 0 if direction == Direction.N or direction == Direction.S else 1

    while math.fabs(world_to_grid_location(current_location)[index] - target_position[index]) > eps:
        globals.pub.publish(move_forward_msg)
        current_location = get_location()
        # time.sleep(0.25)

    globals.pub.publish(stay_put_msg)


def turn_toward(target_orientation_z, eps=0.1):
    rotate_msg_pos = Twist()
    rotate_msg_pos.angular.z = 0.005

    rotate_msg_neg = Twist()
    rotate_msg_neg.angular.z = -0.005

    stay_put_msg = Twist()
    current_angle = np.rad2deg(get_euler_orientation()[2])

    while math.fabs(current_angle - target_orientation_z) > eps:
        globals.pub.publish(rotate_msg_pos if current_angle < target_orientation_z else rotate_msg_neg)
        current_angle = np.rad2deg(get_euler_orientation()[2])
        # print "     (%s)rotation euler (deg): %s" % (globals.robot_name, current_angle)

    globals.pub.publish(stay_put_msg)


def print_location():
    global positions_file
    location = get_location()
    positions.append("(%s) : %f,%f \n" % (globals.robot_name, location[0], location[1]))


def get_euler_orientation():
    (_, rot_quat) = globals.tf_listener.lookupTransform("/map",
                                                           globals.robot_name + "/base_footprint",
                                                           rospy.Time(0))
    rot_euler = tf.transformations.euler_from_quaternion(rot_quat)
    return rot_euler


def get_location():
    """
    :return: The robots location, in base coordinates
    """
    try:
        (trans, _) = globals.tf_listener.lookupTransform("/map",
                                                           globals.robot_name + "/base_footprint",
                                                           rospy.Time(0))
        location = (trans[0], trans[1])
        print "location: %s --> %s" % (str(location), str(world_to_grid_location(location)))
        return location
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        rospy.logerr("Service call failed: %s" % e)


def world_to_grid_location(world_location):
    grid_location = \
        (int(math.floor((world_location[1] - globals.response.map.info.origin.position.y) / globals.robot_size)),
         int(math.floor((world_location[0] - globals.response.map.info.origin.position.x) / globals.robot_size)))
    return grid_location


if __name__ == "__main__":
    globals.__init__()
    main()
