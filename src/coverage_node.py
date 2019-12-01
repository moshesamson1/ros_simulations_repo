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
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from tf import transformations
from turtlesim.msg import Pose

from Coverage import *
from Entities import Direction, deprecated
from globals import Globals

# positions_file = open(globals.absolute_path + "/%s_positions" % globals.robot_name, "a+")
DIRECTION_Z = Direction.Z
positions = []


def pose_callback(pose_msg):
    # Keep track of robot position:
    print "+++position callback+++"
    Globals.position_file.write(pose_msg.position + "\n")

    return


def move(last_d, new_d, distance=0.0, percentage=0.0, use_map=True):
    if not use_map:
        move_blindly(last_d, new_d, distance, percentage)
    else:
        move_smartly(last_d, new_d, percentage)


def move_smartly(last_d, new_d, percentage):
    current_grid_location = world_to_grid_location(get_location())

    print "(%s) <%s> Moving %s -> %s ..." % (Globals.robot_name, current_grid_location, last_d, new_d)
    print "(%s) facing angle: %s" % (Globals.robot_name, np.rad2deg(get_euler_orientation()[2]))

    angle = get_angle_from_directions(last_d, new_d)

    print "(%s) get_euler_orientation()[2]: %s" % (Globals.robot_name, get_euler_orientation()[2])
    print "(%s) angle: %s" % (Globals.robot_name, angle)

    # handle going to specific place...
    turn_toward(np.rad2deg(get_euler_orientation()[2] + angle))
    current_location_grid = world_to_grid_location(get_location())
    move_toward(Entities.Slot(current_location_grid[0], current_location_grid[1]).GoByDirection(new_d), new_d)
    # Done.

    print "Done. <%s> %f" % (world_to_grid_location(get_location()), percentage)


def get_angle_from_directions(last_d, new_d):
    if last_d == Direction.Z:
        if new_d == Direction.N:
            angle = np.pi / 2
        elif new_d == Direction.W:
            angle = np.pi
        elif new_d == Direction.S:
            angle = -np.pi / 2
        elif new_d == Direction.E:
            angle = 0.0
        else:
            print("possible error in get_angle_from_directions. new Direction: ")
            print new_d
            angle = 0.0
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
    return angle


@deprecated
def move_blindly(last_d, new_d, distance, percentage):
    print "(%s) Moving %s -> %s ..." % (Globals.robot_name, last_d, new_d)

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
        Globals.pub.publish(rotate_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angle_vel * (t1 - t0)

    Globals.pub.publish(stay_put_msg)

    current_distance = 0.0
    t0 = rospy.Time.now().to_sec()
    while current_distance < distance:
        Globals.pub.publish(move_forward_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = 0.5 * (t1 - t0)

    Globals.pub.publish(stay_put_msg)

    print "Done. %f" % percentage


def main():
    rospy.init_node('coverage_node', argv=sys.argv)
    Globals.robot_size = 0.35  # meters!
    Globals.robot_name = ''

    get_parameters()

    get_publisher_and_subscriber()

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
        print("Done.")

        print("get path from mst...")
        # get coverage path in the fine grid. using the mst of the coarse grid
        path = get_coverage_path_from_mst(coarse_grid_mst, starting_location_grid)
        print("Done.")

        # ~~~
        last_p = path[0]
        last_d = DIRECTION_Z

        # move the robot along the coverage path
        # ignore first step of the path, as it is the starting position
        print("Run over graph...")
        for p_ind in xrange(1, len(path)):
            p = path[p_ind]

            if last_p.decrease_rows() == p:
                new_d = Direction.S
            elif last_p.increase_cols() == p:
                new_d = Direction.E
            elif last_p.increase_rows() == p:
                new_d = Direction.N
            else:
                new_d = Direction.W

            percentage = float(p_ind) / float(len(path))

            relative_turn_rad = get_angle_from_directions(last_d, new_d)
            init_dir = get_angle_from_directions(Direction.Z, last_d)

            absolute_direction_deg = np.rad2deg(init_dir + relative_turn_rad)
            print("<%s> Moving %s -> %s (%s)" % (percentage, last_p, p, absolute_direction_deg))

            # set orientation and move
            set_orientation(absolute_direction_deg)
            move_toward_correct_direction(p, new_d, absolute_direction_deg)
            location = get_location()
            location_grid = world_to_grid_location(location)
            print("After moving to %s. Location: %s , Grid: %s" % (p, location, location_grid))
            try:
                assert Entities.Slot(location_grid[0], location_grid[1]) == p
            except:
                print ("Aimed for: %s and reached %s" % (p, Entities.Slot(location_grid[0], location_grid[1])))
                exit(-1)

            # move(last_d, new_d, percentage=percentage, use_map=True)
            # print_location()
            last_d, last_p = new_d, p

        positions_file = open(Globals.absolute_path + "/%s_positions" % Globals.robot_name, "a+")
        positions_file.writelines(positions)
        positions_file.close()

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def get_publisher_and_subscriber():
    # Publisher - context is inside namespace
    Globals.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)
    # Listener for pose
    Globals.sub = rospy.Subscriber(Globals.robot_name + "/pose", Pose, pose_callback)


def get_parameters():
    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        Globals.robot_size = float(rospy.get_param('~robot_size'))

    if rospy.has_param('~robot_name'):
        Globals.robot_name = rospy.get_param('~robot_name')

    if rospy.has_param('~init_pos'):
        str_pos = rospy.get_param('~init_pos').split()
        Globals.init_pos = float(str_pos[0]), float(str_pos[1])


def set_orientation(target_orientation_z):
    print("*** set_orientation to %d ***" % target_orientation_z)
    turn_toward(target_orientation_z)
    print "Done set_orientation."


def get_distance_from_slot(s, index):
    """
    Get the distance to the given slot s, in specific axis indicated by index
    :param s: the slot measuring distance from
    :param index: the axis in which the distance is measured.
    :return: the distance
    """
    world_location = get_location()
    distance = math.fabs(s.row - (world_location[1] / 2.0)) if index == 0 else math.fabs(
        s.col - (world_location[0] / 2.0))
    print("(dis): distance: %s" % distance)
    assert distance < 5
    return distance


def is_new_distance_lower(old, new):
    print("old: %s" % old)
    print("new: %s" % new)
    return new < old


@deprecated
def is_world_in_grid_slot(s, index, eps=0.05):
    """
    This method is faulted and should be checked!
    :param s:
    :param index:
    :param eps:
    :return:
    """
    # type: (Entities.Slot, int, float) -> bool
    world_location = get_location()
    return_location = math.fabs(s.row - (world_location[1] / 2.0)) <= eps if index == 0 else math.fabs(
        s.col - (world_location[0] / 2.0)) <= eps
    # print("(is_world_in_grid_slot): world_location: %s" % return_location)
    return return_location


def grid_to_world(target_position):
    # type: (Entities.Slot) -> tuple
    return target_position.col * 2.0, target_position.row * 2.0


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """
    :type (tuple, tuple) -> float
    :param v1:
    :param v2:
    :return:
    """
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def passed_target(d1, d2, d3, d4, d5):
    """
    Check if missed target by THIS much (distance from  target is low then high again.
    The distance should have been high, then go low... and if high again it means that we passed local minima of
    distance.
    [d1: before two steps] -> [d2: before 1 steps] -> [d3: target] -> [d4: after 1 steps] -> [d5: after two steps]
    :param d5:
    :param d4:
    :param d1:
    :param d2:
    :param d3:
    :return:
    """
    print("d1,d2,d3,d4,d5: %s,%s,%s,%s,%s" % (d1, d2, d3, d4, d5))

    cond = d1 >= d2 >= d3 and d3 < d4 < d5
    if cond:
        print("Passed Target!")

    # sanity checks
    assert d5 < 5
    # The following sanity check caused more errors than good, therefore removed for now.
    # if d1 < d2 < d3 < d4 < d5:
    #     print("wrong distances - all in reverse!")
    #     print("d1,d2,d3,d4,d5: %s,%s,%s,%s,%s" % (d1, d2, d3, d4, d5))
    #     exit(-1)

    return cond


def assert_positive_location_fix_otherwise(current_location, current_orientation):
    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.05
    stay_put_msg = Twist()

    if current_location[0] >= -0.5 and current_location[1] >= -0.5:
        return current_location

    fixed_location = current_location

    for i in [0, 1]:
        if current_location[i] < -0.5:  # fix y location
            set_orientation(0.0 if i == 0 else 90.0)
            while fixed_location[i] < 0:
                Globals.pub.publish(move_forward_msg)
                fixed_location = get_location()
                Globals.pub.publish(stay_put_msg)
                print("current location: (%s,%s)" % (fixed_location[0], fixed_location[1]))
            Globals.pub.publish(stay_put_msg)
            set_orientation(current_orientation)

    print("fixing from %s to %s" % (current_location, fixed_location))
    return fixed_location


def move_toward_correct_direction(target_position, direction, target_orientation_z, eps=0.01):
    # type: (Entities.Slot, Direction, float, float) -> None
    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.5
    stay_put_msg = Twist()

    # compute index to compare against. If moving south or north, compare rows. Otherwise compare columns
    index = 0 if (direction == Direction.N or direction == Direction.S) else 1

    # initiate indices
    d1, d2, d3, d4, d5 = 0, 3, 2, 1, round(get_distance_from_slot(target_position, index), 3)

    while not passed_target(d1, d2, d3, d4, d5):
        print("in while..")
        time.sleep(0.01)  # do we need that???

        # move one unit forward
        Globals.pub.publish(move_forward_msg)
        # Globals.pub.publish(stay_put_msg)

        current_location = assert_positive_location_fix_otherwise(get_location(), target_orientation_z)

        # update parameters
        d1, d2, d3, d4, d5 = d2, d3, d4, d5, round(get_distance_from_slot(target_position, index), 3)

        # correct orientation every single whole unit of map
        if current_location[1 if (direction == Direction.N or direction == Direction.S) else 0] % 1.0 < 0.05:
            turn_toward(target_orientation_z, 0.005)

        grid_position = world_to_grid_location(current_location)
        try:
            assert round(grid_position[0]) >= 0
            assert round(grid_position[1]) >= 0
            assert round(grid_position[0]) <= 32
            assert round(grid_position[1]) <= 32
        except:
            print "Assertion Error: value out of range!"
            print(grid_position)
            exit(-1)

    print("Reached target (%s,%s)." % (current_location[0], current_location[1]))


@deprecated
def move_toward(target_position, direction):
    # type: (Entities.Slot, Direction) -> None
    """
    moving from current location to specific location, in a straight line. Always move forward, after robot was rotated.
    :param target_position:
    :param direction:
    :param eps:
    :return:
    """
    print "(%s) move_toward" % Globals.robot_name
    move_forward_msg = Twist()
    move_forward_msg.linear.x = 0.05
    stay_put_msg = Twist()

    current_location = get_location()

    # compute index to compare against. If moving south or north, compare rows. Otherwise compare columns
    index = 1 if direction == Direction.N or direction == Direction.S else 0
    while world_to_grid_location(current_location)[index] != float(target_position[index]):
        Globals.pub.publish(move_forward_msg)
        current_location = get_location()

        print "(%s) current_location: %s, target_location:  %s" % (
            Globals.robot_name, current_location, target_position)
        grid_position = world_to_grid_location(current_location)
        try:
            assert round(grid_position[0]) >= 0
            assert round(grid_position[1]) >= 0
            assert round(grid_position[0]) <= 99
            assert round(grid_position[1]) <= 99
        except:
            print "ERROR: "
            print(grid_position)
            exit(-1)
        # counter += 1

    Globals.pub.publish(stay_put_msg)
    print "Done."


def angle_diff(source, target):
    """
    Return the difference between two angle
    :param source: source angle (deg)
    :param target: target angle (deg)
    :return: The difference (deg)
    """
    a = target - source
    a = (a + 180) % 360 - 180
    return a


def turn_toward(target_orientation_z, eps=0.1):
    """
    Turn toward specific location
    :param target_orientation_z: target to turn to, IN DEGREES!
    :param eps:
    :return: None
    """

    # print "(%s) turning toward %s" % (Globals.robot_name, target_orientation_z)

    rotate_msg_pos = Twist()
    rotate_msg_pos.angular.z = 0.05

    rotate_msg_neg = Twist()
    rotate_msg_neg.angular.z = -0.05

    stay_put_msg = Twist()
    current_angle = np.rad2deg(get_euler_orientation()[2])

    while math.fabs(current_angle % 360.0 - target_orientation_z % 360.0) > eps:
        Globals.pub.publish(rotate_msg_pos if angle_diff(current_angle, target_orientation_z) > 0 else rotate_msg_neg)
        current_angle = np.rad2deg(get_euler_orientation()[2])

    Globals.pub.publish(stay_put_msg)


def log_location_info():
    location = get_location()
    grid_location = world_to_grid_location(location)
    positions.append("(%s) : %f,%f -> %s \n" % (Globals.robot_name, location[0], location[1], grid_location))


def get_euler_orientation():
    """
    Return euler orientation, in radians
    :return:
    """
    (_, rot_quat) = Globals.tf_listener.lookupTransform("/map",
                                                        Globals.robot_name + "/base_footprint",
                                                        rospy.Time(0))
    rot_euler = tf.transformations.euler_from_quaternion(rot_quat)
    return rot_euler


def get_location():
    """
    :return: The robots location, in base coordinates
    """
    try:
        (trans, _) = Globals.tf_listener.lookupTransform("/map",
                                                         Globals.robot_name + "/base_footprint",
                                                         rospy.Time(0))

        location = (round(trans[0]) if too_small_reminder(trans[0]) else trans[0],
                    round(trans[1]) if too_small_reminder(trans[1]) else trans[1])

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
