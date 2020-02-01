#!/usr/bin/python
#
# coverage_node.py
#
#  Created on: today
#      Author: Moshe Samson
#

import sys
import time
from random import seed

import numpy as np
import tf
import tf2_ros
import tqdm
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from tf import transformations
from turtlesim.msg import Pose
from std_msgs.msg import String

from Coverage import *
from Entities import Direction
from globals import Globals
from logs import logStep, logAllSteps

DIRECTION_Z = Direction.Z
positions = []


def status_callback(statusMsg):
    #:type: String
    if str(statusMsg.data)=="finished" and Globals.is_finished:
        Globals.is_terminating = True


def main():
    rospy.init_node('coverage_node', argv=sys.argv)
    Globals.robot_size = 0.35  # meters!
    Globals.robot_name = ''

    get_parameters()

    get_publisher_and_subscriber()

    # if manager, subscribe to event
    if Globals.is_manager:
        rospy.Subscriber("/status", String, status_callback)

    # initiate map service
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        Globals.response = get_static_map()
        rospy.loginfo("Received a width %d X height %d map @ %.3f m/px" % (
            Globals.response.map.info.width, Globals.response.map.info.height, Globals.response.map.info.resolution))
        rospy.loginfo("position: " + str(Globals.response.map.info.origin.position))

        create_occupancy_grid_using_robot_size(Globals.response.map, Globals.robot_size)
        time.sleep(2)
        print "(%s) before tf" % Globals.robot_name
        # find robot's position
        Globals.tf_listener = tf.TransformListener()
        Globals.tf_listener.waitForTransform('/map', Globals.robot_name + "/base_footprint", rospy.Time(0),
                                             rospy.Duration(10))
        print(Globals.tf_listener.getFrameStrings())
        print(Globals.tf_listener.canTransform("/map", Globals.robot_name + "/base_footprint", rospy.Time(0)))
        print "(%s) after tf" % Globals.robot_name

        # point robot to the correct direction
        set_orientation(0.0)
        print "(%s) angle after orientation set: %s" % (Globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

        # compute robot initial position in grid
        starting_location = get_location()
        starting_location_grid = world_to_grid_location(starting_location)

        print "(%s) init pos: %s" % (Globals.robot_name, str(Globals.init_pos))
        print "(%s) starting location: %s" % (Globals.robot_name, str(starting_location))
        print "(%s) starting location grid: %s" % (Globals.robot_name, str(starting_location_grid))

        # switch to coarse grid cell, each cell of size 4D
        switch_to_coarse_grid()
        print "(%s) angle switch_to_coarse_grid set: %s" % (Globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

        starting_location_coarse_grid = int((starting_location_grid[0] - 1) / 2.0), int(
            (starting_location_grid[1] - 1) / 2.0)
        print "(%s) starting_location_coarse_grid: %s" % (Globals.robot_name, str(starting_location_coarse_grid))

        print("get coarse grid mst...")
        coarse_grid_edges = get_edges_from_grid(globals.coarse_grid)
        coarse_grid_graph = create_graph(coarse_grid_edges)
        coarse_grid_mst = mst(starting_location_coarse_grid, coarse_grid_graph)

        print("get path from mst...")
        # get coverage path in the fine grid. using the mst of the coarse grid
        path = get_coverage_path_from_mst(coarse_grid_mst, starting_location_grid)
        print("Done.")

        # move the robot along the coverage path
        # ignore first step of the path, as it is the starting position
        print("Run over graph...")
        for p_ind in tqdm.tqdm(xrange(1, len(path)), position=int(Globals.robot_name[-1]) % 2):
            move_from_x_to_y_using_angle(path[p_ind - 1], path[p_ind])
            logStep(path[p_ind], time.time())

        logAllSteps()

        if Globals.is_manager:
            Globals.is_finished=True
            while not Globals.is_terminating:
                time.sleep(1)
        else:
            print("other node has finished and breaking the bond")

            rate = rospy.Rate(0.5)
            while not rospy.is_shutdown():
                Globals.status_pub.publish("finished")
                time.sleep(1)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def move_from_x_to_y_using_angle(source, target):
    """
    Move from source to target. If missed, stop, recalculate direction and try again until succeeds
    :param source: move from
    :param target: move to
    :return:
    """
    (source_x, source_y) = grid_to_world(source)
    (target_x, target_y) = grid_to_world(target)
    x_to_y_angle = Angle(source_x, source_y, target_x, target_y)
    set_orientation(x_to_y_angle)

    move_forward_msg = Twist()
    move_forward_msg.linear.x = 1.5
    not_reached = True
    max_distance = get_linear_distance_from_slot(target)
    # todo: fix conditions!
    while not_reached:
        Globals.pub.publish(move_forward_msg)  # publish according to distance?
        distance = get_linear_distance_from_slot(target)
        if distance < 0.1:
            # print("REACHED TARGET (%s)!" % target)
            not_reached = False
        elif distance > max_distance or distance > 1.25:
            # print("PASSED TARGET(%s). set angle toward target and try again." % target)
            current_location = get_location()
            current_to_y_angle = Angle(current_location[0], current_location[1], target_x, target_y)
            set_orientation(current_to_y_angle % 360)
        else:
            # print("still moving toward target...")
            pass
        max_distance = distance
    # Globals.pub.publish(stay_put_msg)


def get_publisher_and_subscriber():
    Globals.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)
    Globals.status_pub = rospy.Publisher("/status", String, queue_size=1)
    Globals.status_pub.publish("started")


def get_parameters():
    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        Globals.robot_size = float(rospy.get_param('~robot_size'))

    if rospy.has_param('~robot_name'):
        Globals.robot_name = rospy.get_param('~robot_name')

    if rospy.has_param('~init_pos'):
        str_pos = rospy.get_param('~init_pos').split()
        Globals.init_pos = float(str_pos[0]), float(str_pos[1])

    if rospy.has_param('~is_manager'):
        Globals.is_manager = bool(rospy.get_param('~is_manager'))
        rospy.logwarn(str(Globals.robot_name) + ' is manager? ' + str(Globals.is_manager))


def set_orientation(target_orientation_z):
    # print("*** set_orientation to %d ***" % target_orientation_z)
    turn_toward(target_orientation_z)
    # print "Done set_orientation."


def get_linear_distance_from_slot(slot):
    """
    :type (Entities.Slot)->int
    Get the linear distance from the current position to the given slot s
    :param slot: the slot measuring distance from
    :return: the distance
    """
    world_location = get_location()
    dist_1 = math.pow(slot.row - (world_location[1] / 2.0), 2)
    dist_2 = math.pow(slot.col - (world_location[0] / 2.0), 2)
    distance = math.sqrt(dist_1 + dist_2)
    return distance


def grid_to_world(target_position):
    # type: (Entities.Slot) -> tuple
    return target_position.col * 2.0, target_position.row * 2.0


def turn_toward(target_orientation_z, eps=0.1):
    """
    Turn toward specific location
    :param target_orientation_z: target to turn to, IN DEGREES!
    :param eps:
    :return: None
    """

    angle_diff = lambda source, target: (target - source + 180) % 360 - 180
    sign = lambda x: 1 if x > 0 else -1

    rotate_msg = Twist()
    stay_put_msg = Twist()

    Globals.pub.publish(stay_put_msg)
    current_angle = np.rad2deg(get_euler_orientation()[2])
    while math.fabs(current_angle % 360.0 - target_orientation_z % 360.0) > eps:
        diff = angle_diff(current_angle, target_orientation_z)
        rotate_msg.angular.z = sign(diff)*min(0.2, pow(diff, 8)+0.05)
        Globals.pub.publish(rotate_msg)
        current_angle = np.rad2deg(get_euler_orientation()[2])

    Globals.pub.publish(stay_put_msg)


def get_euler_orientation():
    """
    Return euler orientation, in radians
    :return:
    """
    # t = Globals.tf_listener.getLatestCommonTime("/map", Globals.robot_name + "/base_footprint")
    # Globals.tf_listener.waitForTransform("/map", Globals.robot_name + "/base_footprint",t, rospy.Duration(10))
    # print(Globals.tf_listener.getFrameStrings())
    while not Globals.tf_listener.canTransform("/map", Globals.robot_name + "/base_footprint", rospy.Time(0)):
        time.sleep(0.1)
    t = Globals.tf_listener.getLatestCommonTime("/map", Globals.robot_name + "/base_footprint")
    (_, rot_quat) = Globals.tf_listener.lookupTransform("/map", Globals.robot_name + "/base_footprint", rospy.Time(0))
    rot_euler = tf.transformations.euler_from_quaternion(rot_quat)
    return rot_euler


def get_location():
    """
    :return: The robots location, in base coordinates
    """
    try:
        t = Globals.tf_listener.getLatestCommonTime("/map", Globals.robot_name + "/base_footprint")
        (trans, _) = Globals.tf_listener.lookupTransform("/map", Globals.robot_name + "/base_footprint", t)

        location = (round(trans[0]) if too_small_reminder(trans[0]) else trans[0],
                    round(trans[1]) if too_small_reminder(trans[1]) else trans[1])

        return location
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        rospy.logerr("Service call failed: %s" % e)


def Angle(x1, y1, x2, y2):
    """
    Return the angle of the vector: (x1,y1)--->(x1,y2)
    :param x1: s.e.
    :param y1: s.e.
    :param x2: s.e.
    :param y2: s.e.
    :return: the angle between
    """
    # return angle of line
    # right  = 0; down = pi/2 (90) ; left = pi (180) ; up = 3pi/2 (270)
    if x1 == x2:
        if y2 > y1:
            result = 0.5 * math.pi
        else:
            result = 1.5 * math.pi
        return result

    result = math.atan((y2 - y1) / (x2 - x1))
    if x2 < x1:
        result = result + math.pi

    if result < 0:
        result = result + 2 * math.pi
    result = result * 180 / math.pi
    return result


def too_small_reminder(number, precision=0.0001):
    return math.fabs(number - int(number)) < precision


def world_to_grid_location(world_location):
    grid_x = int(round(world_location[1] / 2.0))
    grid_y = int(round(world_location[0] / 2.0))

    try:
        assert grid_x >= -0.5
        assert grid_y >= -0.5
    except:
        print "Error in world_to_grid_location"
        print grid_x
        print grid_y
        print world_location
        exit(2)

    grid_location = (grid_x, grid_y)
    return grid_location


if __name__ == "__main__":
    # Globals()
    seed(1)
    main()
