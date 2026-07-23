"""Scenario Description:

Approaching a four-way intersection with green traffic signals, the ego vehicle (green box) travels in the left lane with the intention of changing to the right lane to execute a right turn. However, the vehicle is compelled to slow down and wait as its path is obstructed by two adversaries (orange boxes) that are either stationary or moving slowly. One adversary is positioned directly ahead in the ego vehicle's current lane, while the second adversary occupies the adjacent right lane, thereby blocking both the forward trajectory and the necessary lane change maneuver.

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

EGO_INIT_DIST = [30, 40]
param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_FRONT_DIST = [10, 18]
ADV_RIGHT_DIST = [8, 16]
param ADV_SPEED = VerifaiRange(0, 3)

param SAFETY_DIST = VerifaiRange(12, 20)
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

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego starts in an incoming lane and intends a right turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1: directly ahead in ego's current lane
advFrontSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 2: in the adjacent right lane relative to ego's lane
# Find lanes that are to the right of egoInitLane at the same road section
rightLanes = filter(lambda l: l is not egoInitLane and l.road is egoInitLane.road, egoInitLane.road.lanes)
adjacentRightLane = Uniform(*rightLanes) if rightLanes else egoInitLane
advRightSpawnPt = new OrientedPoint in adjacentRightLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with color (0, 1, 0)  # green box

adversaryFront = new Car at advFrontSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=[egoInitLane]),
    with color (1, 0.6, 0)  # orange box

adversaryRight = new Car at advRightSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=[adjacentRightLane]),
    with color (1, 0.6, 0)  # orange box

# Ensure ego is at proper distance from intersection
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]

# Ensure front adversary is ahead of ego at proper distance
require ADV_FRONT_DIST[0] <= (distance from ego to adversaryFront) <= ADV_FRONT_DIST[1]
require (distance along egoInitLane.centerline from egoSpawnPt to advFrontSpawnPt) > 0

# Ensure right adversary is at proper lateral/longitudinal position
require ADV_RIGHT_DIST[0] <= (distance from ego to adversaryRight) <= ADV_RIGHT_DIST[1]

# Terminate after sufficient travel
terminate when (distance from ego to egoSpawnPt) > TERM_DIST