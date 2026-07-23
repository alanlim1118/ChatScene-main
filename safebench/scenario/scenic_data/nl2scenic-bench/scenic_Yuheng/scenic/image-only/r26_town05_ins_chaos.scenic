"""Scenario Description:

In a dense urban area characterized by high-rise buildings and a parking lot visible in the upper left, the ego vehicle proceeds straight through a multi-lane four-way intersection. The scenario depicts a highly complex conflict involving multiple adversaries with intersecting trajectories visualized by colored lines. Specifically, a purple vehicle from the same approach arm executes a left turn, while a green vehicle from the opposing arm, a blue vehicle from the left arm, and a yellow vehicle from the right arm all attempt to pass straight through the intersection, creating a crowded and potentially hazardous traffic situation where multiple paths cross.

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
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

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

# Ego goes straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Purple adversary: same arm as ego, makes left turn
purpleInitLane = egoInitLane
purpleManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, purpleInitLane.maneuvers))
purpleTrajectory = [purpleInitLane, purpleManeuver.connectingLane, purpleManeuver.endLane]
purpleSpawnPt = new OrientedPoint in purpleInitLane.centerline

# Green adversary: opposing arm, goes straight
greenInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers)
    ).startLane
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, greenInitLane.maneuvers))
greenTrajectory = [greenInitLane, greenManeuver.connectingLane, greenManeuver.endLane]
greenSpawnPt = new OrientedPoint in greenInitLane.centerline

# Blue adversary: left arm relative to ego, goes straight
blueInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.leftConflictingManeuvers)
    ).startLane
blueManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, blueInitLane.maneuvers))
blueTrajectory = [blueInitLane, blueManeuver.connectingLane, blueManeuver.endLane]
blueSpawnPt = new OrientedPoint in blueInitLane.centerline

# Yellow adversary: right arm relative to ego, goes straight
yellowInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.rightConflictingManeuvers)
    ).startLane
yellowManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, yellowInitLane.maneuvers))
yellowTrajectory = [yellowInitLane, yellowManeuver.connectingLane, yellowManeuver.endLane]
yellowSpawnPt = new OrientedPoint in yellowInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

purpleAdv = new Car at purpleSpawnPt,
    with blueprint 'vehicle.tesla.model3',
    with color (0.5, 0.0, 0.8),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=purpleTrajectory)

greenAdv = new Car at greenSpawnPt,
    with blueprint 'vehicle.tesla.model3',
    with color (0.0, 0.8, 0.0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=greenTrajectory)

blueAdv = new Car at blueSpawnPt,
    with blueprint 'vehicle.tesla.model3',
    with color (0.0, 0.0, 0.8),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=blueTrajectory)

yellowAdv = new Car at yellowSpawnPt,
    with blueprint 'vehicle.tesla.model3',
    with color (0.9, 0.9, 0.0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=yellowTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from purpleAdv to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from greenAdv to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from blueAdv to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from yellowAdv to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST