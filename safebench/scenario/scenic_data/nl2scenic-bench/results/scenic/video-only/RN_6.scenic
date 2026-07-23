"""Scenario Description:

The ego vehicle, represented by a green box, approaches a roundabout in the right-hand lane of a multi-lane road, traveling alongside an adversary vehicle marked by a yellow box in the adjacent left lane. As the vehicles proceed to enter and navigate the roundabout, the adversary vehicle cuts directly into the ego vehicle's lane from the left. This maneuver forces the ego vehicle to react and adjust its behavior to avoid a collision while continuing through the intersection.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [25, 35]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_INIT_DIST = [25, 35]
param ADV_SPEED = VerifaiRange(7, 10)
param ADV_CUT_SPEED = VerifaiRange(5, 8)

param SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 4
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

behavior AdversaryCutInBehavior(trajectory, cutLane):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    interrupt when (distance to cutLane.centerline) < 15:
        do LaneChangeBehavior(direction='right', target_speed=globalParameters.ADV_CUT_SPEED)
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_CUT_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout intersection
roundabout = Uniform(*filter(lambda i: i.isRoundabout, network.intersections))

# Ego starts in rightmost incoming lane
egoInitLane = Uniform(*filter(lambda l: l.isRightmost, roundabout.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary starts in adjacent left lane (same road section)
advInitLane = Uniform(*filter(lambda l: 
        l.road is egoInitLane.road and l is not egoInitLane and l.leftNeighbor is egoInitLane,
        roundabout.incomingLanes))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (255, 255, 0),
    with behavior AdversaryCutInBehavior(advTrajectory, egoInitLane)

require EGO_INIT_DIST[0] <= (distance from ego to roundabout) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to roundabout) <= ADV_INIT_DIST[1]
require abs((distance from ego to roundabout) - (distance from adversary to roundabout)) < 10

terminate when (distance from ego to egoSpawnPt) > TERM_DIST