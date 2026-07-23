"""Scenario Description:

In a top-down simulation view, a white ego vehicle with black stripes travels straight along a road towards a four-way intersection at a constant speed of 10 km/h. As the ego vehicle approaches the center of the junction, a red vehicle enters from the perpendicular road on the left and initiates a left turn, crossing directly into the ego vehicle's path. The red vehicle fails to yield, resulting in a collision where the front structure of the turning red vehicle strikes the front-left structure of the oncoming ego vehicle in the middle of the intersection.

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
ADV_INIT_DIST = [10, 20]
EGO_SPEED = 10 / 3.6  # 10 km/h in m/s
param ADV_SPEED = VerifaiRange(5, 15)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary enters from the perpendicular road on the left and turns left
advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
            .conflictingManeuvers)
    ).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (1, 1, 1),
    with behavior FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST