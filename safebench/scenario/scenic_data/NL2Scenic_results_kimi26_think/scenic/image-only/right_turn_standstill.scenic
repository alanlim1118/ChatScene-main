"""Scenario Description:

A blue vehicle is positioned at a four-way intersection, depicted in the midst of executing a right turn. A dashed trajectory line extends from the bottom lane, curving towards the right, accompanied by a solid arrow indicating the vehicle's intended path into the perpendicular lane. The scenario description indicates that the vehicle is currently in a standstill state while performing this turning maneuver.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    while True:
        take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoManeuver.connectingLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color [0, 0, 255],
    with behavior EgoBehavior()

terminate after 10 seconds