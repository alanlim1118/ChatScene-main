"""Scenario Description:

The ego vehicle, a blue car, travels straight through a rural four-way intersection surrounded by green vegetation and a large red building in the upper right corner. As the ego vehicle approaches from the bottom and enters the junction, two adversary vehicles, a red car and a grey car, approach from the right arm. Both adversary vehicles execute right turns onto the vertical road, heading downwards away from the intersection, while the ego vehicle continues its upward trajectory through the center of the junction without obstruction.

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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)

ADV_INIT_DIST = [10, 20]
param ADV_SPEED = VerifaiRange(7, 10)

TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: approaches from the bottom and travels straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Identify the opposite incoming lane so we can select the lateral (right) arm
egoStraight = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
oppositeStraight = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStraight.reverseManeuvers))
oppositeLane = oppositeStraight.startLane

# Adversaries: approach from a lateral arm and execute a right turn
lateralLanes = filter(lambda lane: lane is not egoInitLane and lane is not oppositeLane
                      and any(m.type is ManeuverType.RIGHT_TURN for m in lane.maneuvers),
                      intersection.incomingLanes)
advInitLane = Uniform(*lateralLanes)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at advSpawnPt1,
    with blueprint MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary2 = new Car at advSpawnPt2,
    with blueprint MODEL,
    with color (0.5, 0.5, 0.5),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST