#!/usr/bin/python
#
# coverage_node.py
#
#  Created on: today
#      Author: Moshe Samson
#

import sys
from random import seed

import numpy as np
import tf
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from turtlesim.msg import Pose

from Coverage import *
from Entities import Direction, deprecated

# positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
DIRECTION_Z = Direction.Z
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
        move_smartly(last_d, new_d, percentage)


def move_smartly(last_d, new_d, percentage):
    current_grid_location = world_to_grid_location(get_location())

    print "(%s) <%s> Moving %s -> %s ..." % (globals.robot_name, current_grid_location, last_d, new_d)
    print "(%s) facing angle: %s" % (globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

    # todo: use vectors to represent direction and compute angles
    if last_d == Direction.Z:
        if last_d == Direction.E:
            angle = -np.pi / 2
        elif last_d == Direction.W:
            angle = -np.pi
        elif last_d == Direction.S:
            angle = np.pi / 2
        else:
            angle = 0
    else:
        if (last_d == Direction.W and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.E) or \
                (last_d == Direction.E and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.W):
            angle = -np.pi / 2
        elif (last_d == Direction.W and new_d == Direction.S) or (last_d == Direction.S and new_d == Direction.E) or \
                (last_d == Direction.E and new_d == Direction.N) or (last_d == Direction.N and new_d == Direction.W):
            angle = np.pi / 2
        elif last_d == new_d:
            angle = 0
        else:
            rospy.logerr("Wrong directions. trying to switch from %s to %s" % (last_d, new_d))
            exit(-1)

    print "(%s) get_euler_orientation()[2]: %s" % (globals.robot_name, get_euler_orientation()[2])
    print "(%s) angle: %s" % (globals.robot_name, angle)

    # handle going to specific place...
    turn_toward(np.rad2deg(get_euler_orientation()[2] + angle))
    current_location_grid = world_to_grid_location(get_location())
    move_toward(Entities.Slot(current_location_grid[0], current_location_grid[1]).GoByDirection(new_d), new_d)
    # Done.

    print "Done. <%s> %f" % (world_to_grid_location(get_location()), percentage)


@deprecated
def move_blindly(last_d, new_d, distance, percentage):
    print "(%s) Moving %s -> %s ..." % (globals.robot_name, last_d, new_d)

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

    get_parameters()

    get_publisher_and_subscriber()

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
        set_orientation(90.0)
        print "(%s) angle after orientation set: %s" % (globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

        # compute robot initial position in grid
        starting_location = get_location()
        starting_location_grid = world_to_grid_location(starting_location)

        print "(%s) init pos: %s" % (globals.robot_name, str(globals.init_pos))
        print "(%s) starting location: %s" % (globals.robot_name, str(starting_location))
        print "(%s) starting location grid: %s" % (globals.robot_name, str(starting_location_grid))

        # switch to coarse grid cell, each cell of size 4D
        switch_to_coarse_grid()
        print "(%s) angle switch_to_coarse_grid set: %s" % (globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

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
        last_d = DIRECTION_Z

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

        positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
        positions_file.writelines(positions)
        positions_file.close()

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def get_publisher_and_subscriber():
    # Publisher - context is inside namespace
    globals.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)
    # Listener for pose
    globals.sub = rospy.Subscriber(globals.robot_name + "/pose", Pose, pose_callback)


def get_parameters():
    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        globals.robot_size = float(rospy.get_param('~robot_size'))

    if rospy.has_param('~robot_name'):
        globals.robot_name = rospy.get_param('~robot_name')
        if globals.robot_name == 'robot_1':
            exit(2)

    if rospy.has_param('~init_pos'):
        str_pos = rospy.get_param('~init_pos').split()
        globals.init_pos = float(str_pos[0]), float(str_pos[1])


def set_orientation(target_orientation_z):
    print "*** set_orientation ***"
    turn_toward(target_orientation_z)
    print "Done."


def move_toward(target_position, direction, eps=0.1):
    # type: (Entities.Slot, Direction, float) -> None
    """
    moving from current location to specific location, in a straight line. Always move forward, after robot was rotated.
    :param target_position:
    :param direction:
    :param eps:
    :return:
    """
    print "(%s) move_toward" % globals.robot_name
    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.5

    stay_put_msg = Twist()

    current_location = get_location()

    # compute index to compare against. If moving south or north, compare rows. Otherwise compare columns
    counter = 0
    index = 0 if direction == Direction.N or direction == Direction.S else 1
    while math.fabs(world_to_grid_location(current_location)[index] - target_position[index]) > eps:
        globals.pub.publish(move_forward_msg)
        current_location = get_location()\

        grid_position = world_to_grid_location(current_location)
        # print "(%s) grid_position: %s" % (globals.robot_name, grid_position)
        assert grid_position[0] >= 0
        assert grid_position[1] >= 0
        assert grid_position[0] <= 99
        assert grid_position[1] <= 99
        # counter += 1

        # if counter >= 10000:
        #     print "+++(%s)target_position: %s" % (globals.robot_name, target_position)
        #     print "+++(%s)direction: %s" % (globals.robot_name, direction)
        #     exit(-1)

    globals.pub.publish(stay_put_msg)
    print "Done."


def turn_toward(target_orientation_z, eps=0.1):
    """
    Turn toward specific location
    :param target_orientation_z: target to turn to, IN DEGREES!
    :param eps:
    :return: None
    """

    print "(%s) turning toward %s" % (globals.robot_name, target_orientation_z)

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
    globals.pub.publish(stay_put_msg)


def print_location():
    global positions_file
    location = get_location()
    positions.append("(%s) : %f,%f \n" % (globals.robot_name, location[0], location[1]))


def get_euler_orientation():
    """
    Return euler orientation, in radians
    :return:
    """
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

        location = (int(trans[0]) if too_small_reminder(trans[0]) else trans[0],
                    int(trans[1]) if too_small_reminder(trans[1]) else trans[1])

        if location[0] < -1:
            print "location < -1"
            print location
            print trans

        return location
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        rospy.logerr("Service call failed: %s" % e)


def too_small_reminder(number, precision=0.0001):
    return math.fabs(number - int(number)) < precision


def world_to_grid_location(world_location):

    # grid_x = int(math.floor((globals.response.map.info.height - (world_location[1] - globals.response.map.info.origin.position.y)) / globals.robot_size))
    # grid_y = int(math.floor((world_location[0] - globals.response.map.info.origin.position.x) / globals.robot_size))

    grid_x = int(math.floor(world_location[1] / globals.robot_map_size))
    grid_y = int(math.floor(world_location[0] / globals.robot_map_size))

    grid_location = (grid_x, grid_y)
    return grid_location


if __name__ == "__main__":
    globals.__init__()
    seed(1)
    main()
