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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_RED_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_GREY_MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(6, 9)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego approaches from bottom going straight up
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries approach from the right arm (relative to ego's incoming direction)
# The right arm relative to ego's straight maneuver corresponds to lanes whose
# straight maneuvers are conflicting with ego's straight maneuver from the right side
rightArmLanes = filter(lambda m:
    m.type is ManeuverType.STRAIGHT,
    egoManeuver.conflictingManeuvers
)
advInitLane = Uniform(*[m.startLane for m in rightArmLanes])

# Both adversaries make right turns onto the vertical road heading downward
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 255),
    with behavior EgoBehavior(egoTrajectory)

adversary_red = new Car at advSpawnPt1,
    with blueprint ADV_RED_MODEL,
    with color (255, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary_grey = new Car at advSpawnPt2,
    with blueprint ADV_GREY_MODEL,
    with color (128, 128, 128),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary_red to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary_grey to intersection) <= ADV_INIT_DIST[1]
require distance between adversary_red and adversary_grey >= 8

terminate when (distance from ego to egoSpawnPt) > TERM_DIST