from globals import Globals
import os

positions = []


def logAllSteps():
    gazebo_port = str(os.environ['GAZEBO_MASTER_URI']).split(':')[-1]
    ros_port = str(os.environ['ROS_MASTER_URI']).split(':')[-1]
    positions_file = open("%s/%s_%s_%s_positions" %(Globals.absolute_path, ros_port, gazebo_port, Globals.robot_name), "a+")
    positions_file.writelines(positions)
    positions_file.close()


def logStep(slotPosition, coverageTime):
    positions.append("%s:%s\n" % (slotPosition, coverageTime))
