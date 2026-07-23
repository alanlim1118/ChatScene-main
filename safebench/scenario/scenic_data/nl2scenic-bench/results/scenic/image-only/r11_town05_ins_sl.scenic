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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [20, 30]
param ADV_SPEED = VerifaiRange(7, 10)

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

# Select a 4-way intersection matching the described urban layout
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: eastbound incoming lane making a left turn (to go north)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: oncoming vehicle going straight through the intersection
# The conflicting straight maneuver from the opposite direction
advManeuver = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (1.0, 0.0, 0.0),  # Red car
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color (0.0, 0.0, 1.0),  # Blue car
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST