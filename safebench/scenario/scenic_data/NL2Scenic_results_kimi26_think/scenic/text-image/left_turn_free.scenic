"""Scenario Description:

A blue ego vehicle is positioned in the southbound lane of a four-way intersection, facing north, and is shown executing a left turn. A curved blue arrow traces the vehicle's path from its current lane, sweeping leftward across the intersection to merge into the westbound lane. The road layout includes white dashed lane markings separating the lanes and solid white lines marking the intersection boundaries, with gray rectangular areas occupying the four corners. The scenario depicts a clear, unobstructed maneuver where the ego car performs a free left turn at the junction.
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

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(5, 10)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 0, 255),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST