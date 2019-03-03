import os

#todo: create class og globals

def __init__():
    global robot_name, robot_size, robot_map_size
    global position_file
    global pub, sub
    global grid, coarse_grid
    global tf_listener
    global absolute_path
    global init_pos
    global response

    absolute_path = os.path.dirname(os.path.realpath(__file__))
