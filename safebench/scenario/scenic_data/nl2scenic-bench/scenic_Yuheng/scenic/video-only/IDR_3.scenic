"""Scenario Description:

The ego vehicle travels straight through a four-way urban intersection, passing between tall buildings on either side. As the ego vehicle moves through the junction, a red adversary vehicle approaches from the left cross-street and executes a left turn onto the same road ahead of the ego vehicle. The red vehicle completes the turn and proceeds northbound in the lane directly in front of the ego vehicle's path. Another white vehicle is visible waiting at the stop line on the left cross-street behind the turning red car. The ego vehicle continues forward, maintaining its lane as it follows the red vehicle that has just merged onto the roadway.

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

MODEL_EGO = 'vehicle.lincoln.mkz_2017'
MODEL_ADV_RED = 'vehicle.tesla.model3'
MODEL_ADV_WHITE = 'vehicle.audi.a2'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
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

# Ego goes straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the left cross-street and turns left onto ego's outgoing road
# The left cross-street relative to ego is identified via conflicting straight maneuvers
leftCrossLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers)
    ).startLane

# The adversary makes a left turn from the left cross-street
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, leftCrossLane.maneuvers))
advTrajectory = [leftCrossLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in leftCrossLane.centerline

# Static white vehicle waiting behind the red adversary on the same left cross-street
staticSpawnPt = new OrientedPoint in leftCrossLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL_EGO,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL_ADV_RED,
    with color (1.0, 0.0, 0.0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

staticCar = new Car at staticSpawnPt,
    with blueprint MODEL_ADV_WHITE,
    with color (1.0, 1.0, 1.0),
    with behavior StationaryBehavior()

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure the static white car is behind the red adversary on the same lane
require (distance from staticCar to adversary) < 15
require (distance from staticCar to intersection) > (distance from adversary to intersection)

# Ensure adversary ends up in the same outgoing lane as ego
require advManeuver.endLane is egoManeuver.endLane

terminate when (distance to egoSpawnPt) > TERM_DIST