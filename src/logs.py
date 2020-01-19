from globals import Globals

positions = []


def logAllSteps():
    positions_file = open(Globals.absolute_path + "/%s_positions" % Globals.robot_name, "a+")
    positions_file.writelines(positions)
    positions_file.close()


def logStep(slotPosition, coverageTime):
    positions.append("slot:%s---time:%s\n" % (slotPosition, coverageTime))
