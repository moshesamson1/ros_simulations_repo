from random import shuffle
import math
import Entities
import rospy
import globals


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


def create_occupancy_grid(my_map):
    # creating the occupancy grid
    globals.grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                globals.grid[i][j] = False
            else:
                globals.grid[i][j] = True
    return globals.grid


def create_occupancy_grid_using_robot_size(my_map, robot_size):
    rospy.loginfo("create_occupancy_grid_using_d")

    robot_radius = robot_size
    robot_map_size = int(math.ceil(robot_radius / my_map.info.resolution))
    print "robot_map_size: " + str(robot_map_size)

    globals.grid = [[None] * int(math.ceil(my_map.info.width / robot_map_size)) for _ in xrange(int(math.ceil(my_map.info.height / robot_map_size)))]

    print "****** grid size: {0} X {1}".format(len(globals.grid), len(globals.grid[0]))

    for i in xrange(0, my_map.info.height - robot_map_size, robot_map_size):
        for j in xrange(0, my_map.info.width - robot_map_size, robot_map_size):
            globals.grid[i / robot_map_size][j / robot_map_size] = False

            # if any sub-cell is occupied, then the whole cell considered as occupied.
            occupied_sub_cell_found = False
            for row in xrange(i, i + robot_map_size):
                if occupied_sub_cell_found: break
                for col in xrange(j, j + robot_map_size):
                    if my_map.data[row * my_map.info.width + col] > 0:
                        print "Should not reach this position. map should be totally free!"
                        exit(-2)
                        globals.grid[i / robot_map_size][j / robot_map_size] = True
                        occupied_sub_cell_found = True
                        break
    return globals.grid


def switch_to_coarse_grid():
    orig_grid_rows = len(globals.grid)
    orig_grid_cols = len(globals.grid[0])
    globals.coarse_grid = [[None] * (orig_grid_cols / 2) for _ in xrange(orig_grid_rows / 2)]
    for i in xrange(0, orig_grid_rows - 1, 2):
        for j in xrange(0, orig_grid_cols - 1, 2):
            if globals.grid[i][j] or globals.grid[i + 1][j] or globals.grid[i][j + 1] or globals.grid[i + 1][j + 1]:
                globals.coarse_grid[i / 2][j / 2] = True
            else:
                globals.coarse_grid[i / 2][j / 2] = False

    return globals.coarse_grid


def create_adjacency_matrix(grid, ignore_occupancies):
    adjacency_matrix = {}
    for row in xrange(0, len(grid)):
        for col in xrange(0, len(grid[0])):
            to_append = []
            if ignore_occupancies or not grid[row][col]:
                if row < len(grid) - 1 and (ignore_occupancies or not grid[row + 1][col]): to_append.append(
                    (row + 1, col))
                if col < len(grid[0]) - 1 and (ignore_occupancies or not grid[row][col + 1]): to_append.append(
                    (row, col + 1))
                if 0 < row <= len(grid) - 1 and (ignore_occupancies or not grid[row - 1][col]): to_append.append(
                    (row - 1, col))
                if 0 < col <= len(grid[0]) - 1 and (ignore_occupancies or not grid[row][col - 1]): to_append.append(
                    (row, col - 1))
            adjacency_matrix[(row, col)] = set(to_append)
    return adjacency_matrix


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
