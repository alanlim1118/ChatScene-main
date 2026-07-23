"""Scenario Description:

The ego vehicle, shown as a red car, travels eastbound from the left side of a multi-lane, four-way urban intersection and attempts a left turn to proceed north, following a pink trajectory path that curves across the junction. This maneuver places the ego vehicle on a collision course with an oncoming blue adversary vehicle traveling straight southbound from the top, as indicated by its straight blue trajectory line. The surrounding environment features a residential complex with white buildings and green lawns in the upper left, a public plaza with a circular fountain in the upper right, a grassy area with trees in the lower left, and a zone with colorful rectangular structures in the lower right, framing the critical interaction where the turning vehicle crosses the path of the straight-moving traffic.

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

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego vehicle performs a left turn across the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary travels straight through the intersection on a conflicting path
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
