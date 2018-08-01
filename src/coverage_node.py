#!/usr/bin/python
#
# coverage_node.py
#
#  Created on: today
#      Author: Moshe Samson
#

import math
import sys
from random import shuffle
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from turtlesim.msg import Pose

import Entities


def pose_callback(pose_msg):
    # Do nothing
    return

def create_occupancy_grid(my_map):
    # creating the occupancy grid
    global grid
    grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True
    return grid


def create_occupancy_grid_using_robot_size(my_map, robot_size):
    rospy.loginfo("create_occupancy_grid_using_d")

    robot_radius = robot_size
    robot_map_size = int(math.ceil(robot_radius / my_map.info.resolution))
    print "robot_map_size: " + str(robot_map_size)

    global grid
    grid = [[None] * int(math.ceil(my_map.info.width / robot_map_size)) for i in
            xrange(int(math.ceil(my_map.info.height / robot_map_size)))]

    print "****** grid size: {0} X {1}".format(len(grid), len(grid[0]))

    for i in xrange(0, my_map.info.height - robot_map_size, robot_map_size):
        for j in xrange(0, my_map.info.width - robot_map_size, robot_map_size):
            grid[i / robot_map_size][j / robot_map_size] = False

            # if any sub-cell is occupied, then the whole cell considered as occupied.
            occupied_subcell_found = False
            for row in xrange(i, i + robot_map_size):
                if occupied_subcell_found: break
                for col in xrange(j, j + robot_map_size):
                    if my_map.data[row * my_map.info.width + col] > 0:
                        grid[i / robot_map_size][j / robot_map_size] = True
                        occupied_subcell_found = True
                        break
    return grid


def switch_to_coarse_grid():
    print "switch_to_coarse_grid"
    global grid, coarse_grid
    orig_grid_rows = len(grid)
    orig_grid_cols = len(grid[0])
    coarse_grid = [[None] * (orig_grid_cols / 2) for i in xrange(orig_grid_rows / 2)]
    for i in xrange(0, orig_grid_rows - 1, 2):
        for j in xrange(0, orig_grid_cols - 1, 2):
            if grid[i][j] or grid[i + 1][j] or grid[i][j + 1] or grid[i + 1][j + 1]:
                coarse_grid[i / 2][j / 2] = True
            else:
                coarse_grid[i / 2][j / 2] = False

    return coarse_grid


def create_adjacency_matrix(grid, ignore_occupencies):
    adjacency_matrix = {}
    for row in xrange(0, len(grid)):
        for col in xrange(0, len(grid[0])):
            to_append = []
            if ignore_occupencies or not grid[row][col]:
                if row < len(grid) - 1 and (ignore_occupencies or not grid[row + 1][col]): to_append.append(
                    (row + 1, col))
                if col < len(grid[0]) - 1 and (ignore_occupencies or not grid[row][col + 1]): to_append.append(
                    (row, col + 1))
                if 0 < row <= len(grid) - 1 and (ignore_occupencies or not grid[row - 1][col]): to_append.append(
                    (row - 1, col))
                if 0 < col <= len(grid[0]) - 1 and (ignore_occupencies or not grid[row][col - 1]): to_append.append(
                    (row, col - 1))
            adjacency_matrix[(row, col)] = set(to_append)
    return adjacency_matrix


def dfs(graph, start):
    visited, stack = set(), [start]
    while stack:
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph[vertex] - visited)
    return visited


def bfs(graph, start):
    visited, queue = [], [start]
    while queue:
        vertex = queue.pop(0)
        if vertex not in visited:
            if vertex not in visited:
                visited.append(vertex)
            queue.extend(graph[vertex] - set(visited))
    return visited


def get_reachable_cells(_grid, starting_location):
    # create adjacency matrix
    graph = create_adjacency_matrix(_grid, False)
    # for p in sorted(graph.keys(), key=lambda tup: tup[0]):
    #     print str(p) + "::" + str(graph[p])
    connected_component = dfs(graph, starting_location)
    return connected_component


def get_free_starting_location(_coarse_grid, _starting_location_coarse_grid):
    graph = create_adjacency_matrix(_coarse_grid, True)
    bfs_vertices = bfs(graph, _starting_location_coarse_grid)
    return next((x for x in bfs_vertices if not _coarse_grid[x[0]][x[1]]), -1)


def get_edges_from_grid(_grid):
    """ Create list of all the edges. Does not consider direction. i.e. (1,2)->(1,3) and (1,3)->(1,2) are the same.
    :rtype: list
    """
    edges = []
    for row in xrange(0, len(_grid) - 1):
        for col in xrange(0, len(_grid[0]) - 1):
            if not _grid[row][col + 1]: edges.append(((row, col), (row, col + 1)))
            if not _grid[row + 1][col]: edges.append(((row, col), (row + 1, col)))

    return edges


def create_graph(edgelist):
    graph = {}
    for e1, e2 in edgelist:
        graph.setdefault(e1, []).append(e2)
        graph.setdefault(e2, []).append(e1)
    return graph


# Prim's
def mst(start, _graph):
    closed = set()
    edges = []
    q = [(start, start)]
    while q:
        # randomize
        shuffle(q)

        v1, v2 = q.pop()
        if v2 in closed:
            continue
        closed.add(v2)
        edges.append((v1, v2))
        for v in _graph[v2]:
            if v in _graph:
                q.append((v2, v))
    del edges[0]
    # print len(edges)
    # print len(_graph)
    # assert len(edges) == len(_graph) - 1
    return edges


def print_graph(edges):
    import matplotlib.pyplot as plt
    import networkx as nx
    plt.figure()
    g = nx.DiGraph()
    g.add_edges_from(edges)
    positions = {}
    color_map = []
    for n in g.nodes():
        positions[n] = (n[0], n[1])
        color_map.append('green' if n == edges[0][0] else 'red')
    nx.draw(g, pos=positions, node_size=60, font_size=8, node_color=color_map)
    plt.show()


def create_covering_path(mst_edges_shallow_graph, initial_slot):
    covering_path = []
    origin_slot = initial_slot
    slot = origin_slot
    counter = 0
    while True:
        counter += 1
        if counter > 100000:
            print "ERROR!!!"
            return

        # remove after finishing debugging
        # if slot in covering_path:
        #    print "slot already in! Check parameters"

        covering_path.append(slot)
        shallow_slot = Entities.Slot(math.floor(slot.row / 2.0), math.floor(slot.col / 2.0))
        # find to where to go next, depend on the mst edges.
        # Check how much and which corners are in the mst group, then update slot accordingly

        has_downward_edge = (shallow_slot, shallow_slot.GoDown()) in mst_edges_shallow_graph or (
                                                                                                    shallow_slot.GoDown(),
                                                                                                    shallow_slot) in mst_edges_shallow_graph
        has_rightward_edge = (shallow_slot, shallow_slot.GoRight()) in mst_edges_shallow_graph or (
                                                                                                      shallow_slot.GoRight(),
                                                                                                      shallow_slot) in mst_edges_shallow_graph
        has_leftward_edge = (shallow_slot, shallow_slot.GoLeft()) in mst_edges_shallow_graph or (
                                                                                                    shallow_slot.GoLeft(),
                                                                                                    shallow_slot) in mst_edges_shallow_graph
        has_upward_edge = (shallow_slot, shallow_slot.GoUp()) in mst_edges_shallow_graph or (shallow_slot.GoUp(),
                                                                                             shallow_slot) in mst_edges_shallow_graph

        bl_corner_in_mst = False
        br_corner_in_mst = False
        ul_corner_in_mst = False
        ur_corner_in_mst = False

        if slot.row % 2 == 0 and slot.col % 2 == 0:
            if has_downward_edge or has_rightward_edge:
                br_corner_in_mst = True
            if has_upward_edge:
                br_corner_in_mst = True
                ur_corner_in_mst = True
            if has_leftward_edge:
                bl_corner_in_mst = True
                br_corner_in_mst = True
        elif slot.row % 2 == 0 and slot.col % 2 != 0:
            if has_downward_edge or has_leftward_edge:
                bl_corner_in_mst = True
            if has_upward_edge:
                bl_corner_in_mst = True
                ul_corner_in_mst = True
            if has_rightward_edge:
                bl_corner_in_mst = True
                br_corner_in_mst = True
        elif slot.row % 2 != 0 and slot.col % 2 == 0:
            if has_rightward_edge or has_upward_edge:
                ur_corner_in_mst = True
            if has_downward_edge:
                br_corner_in_mst = True
                ur_corner_in_mst = True
            if has_leftward_edge:
                ur_corner_in_mst = True
                ul_corner_in_mst = True
        elif slot.row % 2 != 0 and slot.col % 2 != 0:
            if has_leftward_edge or has_upward_edge:
                ul_corner_in_mst = True
            if has_downward_edge:
                bl_corner_in_mst = True
                ul_corner_in_mst = True
            if has_rightward_edge:
                ul_corner_in_mst = True
                ur_corner_in_mst = True

        last_slot = covering_path[len(covering_path) - 2]

        # check o see if only one vertex was in the mst
        if br_corner_in_mst and not (ur_corner_in_mst or bl_corner_in_mst or ul_corner_in_mst):
            if slot.GoDown() == last_slot or slot == last_slot:
                slot = slot.GoRight()
            elif slot.GoRight() == last_slot:
                slot = slot.GoDown()
        elif ur_corner_in_mst and not (br_corner_in_mst or bl_corner_in_mst or ul_corner_in_mst):
            if slot.GoRight() == last_slot or slot == last_slot:
                slot = slot.GoUp()
            elif slot.GoUp() == last_slot:
                slot = slot.GoRight()
        elif bl_corner_in_mst and not (ul_corner_in_mst or ur_corner_in_mst or br_corner_in_mst):
            if slot.GoLeft() == last_slot or slot == initial_slot:
                slot = slot.GoDown()
            elif slot.GoDown() == last_slot:
                slot = slot.GoLeft()
        elif ul_corner_in_mst and not (bl_corner_in_mst or br_corner_in_mst or ur_corner_in_mst):
            if slot.GoUp() == last_slot or slot == initial_slot:
                slot = slot.GoLeft()
            elif slot.GoLeft() == last_slot:
                slot = slot.GoUp()
        # check to see if exactly two vertices are in the mst
        elif bl_corner_in_mst and br_corner_in_mst and not (ul_corner_in_mst or ur_corner_in_mst):
            if slot.GoLeft() == last_slot or slot == initial_slot:
                slot = slot.GoRight()
            elif slot.GoRight() == last_slot:
                slot = slot.GoLeft()
            else:
                print "error 1"
        elif ul_corner_in_mst and ur_corner_in_mst and not (bl_corner_in_mst or br_corner_in_mst):
            if slot.GoRight() == last_slot or slot == initial_slot:
                slot = slot.GoLeft()
            elif slot.GoLeft() == last_slot:
                slot = slot.GoRight()
        elif br_corner_in_mst and ur_corner_in_mst and not (bl_corner_in_mst or ul_corner_in_mst):
            if slot.GoDown() == last_slot or slot == initial_slot:
                slot = slot.GoUp()
            elif slot.GoUp() == last_slot:
                slot = slot.GoDown()
        elif bl_corner_in_mst and ul_corner_in_mst and not (br_corner_in_mst or ur_corner_in_mst):
            if slot.GoUp() == last_slot or slot == initial_slot:
                slot = slot.GoDown()
            elif slot.GoDown() == last_slot:
                slot = slot.GoUp()
        # check for exactly 3 vertices
        elif br_corner_in_mst and bl_corner_in_mst and ul_corner_in_mst and not ur_corner_in_mst:
            if slot.GoUp() == last_slot or slot == initial_slot:
                slot = slot.GoRight()
            elif slot.GoRight() == last_slot:
                slot = slot.GoUp()
        elif bl_corner_in_mst and ul_corner_in_mst and ur_corner_in_mst and not br_corner_in_mst:
            if slot.GoDown() == last_slot or slot == initial_slot:
                slot = slot.GoRight()
            elif slot.GoRight() == last_slot:
                slot = slot.GoDown()
        elif ul_corner_in_mst and ur_corner_in_mst and br_corner_in_mst and not bl_corner_in_mst:
            if slot.GoLeft() == last_slot or slot == initial_slot:
                slot = slot.GoDown()
            elif slot.GoDown() == last_slot:
                slot = slot.GoLeft()
        elif ur_corner_in_mst and br_corner_in_mst and bl_corner_in_mst and not ul_corner_in_mst:
            if slot.GoUp() == last_slot or slot == initial_slot:
                slot = slot.GoLeft()
            elif slot.GoLeft() == last_slot:
                slot = slot.GoUp()
        else:
            print "error has occured!"

        if slot == origin_slot:
            break

    return covering_path


def get_coverage_path_from_mst(mst, start):
    # convert cells as tuples to Slots (from cell entities)
    mst_edges_as_slots = [(Entities.Slot(edge[0][0], edge[0][1]), Entities.Slot(edge[1][0], edge[1][1])) for edge in
                          mst]

    coverage_path = create_covering_path(mst_edges_as_slots, Entities.Slot(start[0], start[1]))
    return coverage_path
    # PrintGraph(itemlist)


def move_blindly(last_d, new_d, distance):
    global robot_name, pub, sub
    print "Moving " + new_d + "...",

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
        pub.publish(rotate_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angle_vel * (t1 - t0)
    pub.publish(stay_put_msg)

    current_distance = 0.0
    t0 = rospy.Time.now().to_sec()
    while current_distance < distance:
        pub.publish(move_forward_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = 0.5 * (t1 - t0)

    pub.publish(stay_put_msg)

    print "Done."


def main():
    rospy.init_node('coverage_node', argv=sys.argv)
    global grid, coarse_grid, robot_size, robot_name, pub, sub
    starting_location = (0, 0)
    robot_size = 0.35  # meters!
    robot_name = ''

    # get initial parameters from launch file:
    if rospy.has_param('~robot_size'):
        rospy.loginfo("has robot size!")

    if rospy.has_param('~robot_name'):
        robot_name = rospy.get_param('~robot_name')
        print "robot_name: {0}".format(robot_name)

    # Publisher - context is inside namespace
    pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)


    # Listener for pose
    sub = rospy.Subscriber(robot_name + "/pose", Pose, pose_callback)

    # initiate map service
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a width %d X height %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))
        rospy.loginfo("position: " + str(response.map.info.origin.position))

        create_occupancy_grid_using_robot_size(response.map, robot_size)

        # find robot's position
        # listener = tf.TransformListener()
        # listener.waitForTransform('robot_0_ns/map', 'robot_0/base_footprint', rospy.Time(0), rospy.Duration(10))
        #
        # try:
        #     (trans, rot) = listener.lookupTransform("/map", robot_name + "/base_footprint", rospy.Time(0))
        #     starting_location = (int(trans[0]), int(trans[1]))
        #     rospy.loginfo("++++++++++++++++++++++++++++++++++++++++++++current position (%f,%f)" % (starting_location[0], starting_location[1]))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        #     rospy.logerr("Service call failed: %s" % e)




        # # compute robot initial position in grid
        # starting_location_grid = (int(math.floor(
        #     (starting_location[1] - response.map.info.origin.position.y) / robot_size)),
        #                           int(math.floor(
        #                               (starting_location[0] - response.map.info.origin.position.x) / robot_size))
        # )

        starting_location_grid = (len(grid)/2, len(grid[0])/2)
        print "starting location grid: " + str(starting_location_grid)

        # switch to coarse grid cell, each cell of size 4D
        switch_to_coarse_grid()

        # find free initial location in the coarse grid
        starting_location_coarse_grid = (starting_location_grid[0] / 2, starting_location_grid[1] / 2)
        print "starting location coarse grid: " + str(starting_location_coarse_grid)
        if coarse_grid[starting_location_coarse_grid[0]][starting_location_coarse_grid[1]]:
            rospy.loginfo("Original Starting location is occupied! Finding the closest free starting location...")
            # perform bfs over all edges, then get the first one which isn't occupied
            starting_location_coarse_grid = get_free_starting_location(coarse_grid, starting_location_coarse_grid)
            rospy.loginfo("starting location coarse grid: " + str(starting_location_coarse_grid))

        # Find all the free cells that are reachable from the robot's initial position and ignore all the other cells.
        reachable_cells_coarse_grid = get_reachable_cells(coarse_grid, starting_location_coarse_grid)

        # Make sure all unreachable cells considered as occupied.
        for row in xrange(len(coarse_grid)):
            for col in xrange(len(coarse_grid[0])):
                if (row, col) not in reachable_cells_coarse_grid:
                    coarse_grid[row][col] = True

        # create spanning tree of robot
        coarse_grid_edges = get_edges_from_grid(coarse_grid)
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

                succeeded = move_blindly(last_d, new_d, robot_size)
                last_d = new_d

                # succeeded = send_goals(
                #     int(last_p.col * robot_size + response.map.info.origin.position.x),
                #     int(last_p.row * robot_size + response.map.info.origin.position.y))

                last_p = p

        finish_time = rospy.Time.now()
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

    # print_map_to_file()


if __name__ == "__main__":
    main()
