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
BLUE_COLOR = (0, 0, 255)

EGO_INIT_DIST = [15, 25]
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior StandstillRightTurnBehavior(trajectory):
    # Vehicle remains at standstill as specified in scenario description
    take SetThrottleAction(0), SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color BLUE_COLOR,
    with behavior StandstillRightTurnBehavior(egoTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST