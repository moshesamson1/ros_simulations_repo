import os


class Globals:
    robot_name = None
    robot_size = None
    robot_map_size = None
    position_file = None
    pub = None
    sub = None
    grid = None
    coarse_grid = None
    tf_listener = None
    absolute_path = os.path.dirname(os.path.realpath(__file__))
    init_pos = None
    response = None

    def __init__(self):
        pass
