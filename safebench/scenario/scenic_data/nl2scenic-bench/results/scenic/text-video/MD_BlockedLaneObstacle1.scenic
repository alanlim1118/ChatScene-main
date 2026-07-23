"""Scenario Description:

Captured from a top-down aerial perspective, the ego vehicle travels straight along a multi-lane urban street flanked by high-rise buildings under clear weather conditions. As the ego vehicle approaches a four-way intersection, it is forced to wait because a line of dark adversary vehicles is stopped directly ahead in the ego's driving lane, obstructing the path. Meanwhile, in the adjacent right lane, traffic flows freely; a white vehicle and a red vehicle are observed driving straight ahead through the intersection, bypassing the congestion in the left lane. The scene highlights the contrast between the blocked ego lane and the open adjacent lane, with shadows from the buildings and roadside trees cast across the asphalt.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
DARK_VEHICLE_MODELS = [
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.mercedes.coupe",
    "vehicle.mini.cooper_s",
    "vehicle.nissan.patrol"
]
WHITE_VEHICLE_MODEL = "vehicle.tesla.model3"
RED_VEHICLE_MODEL = "vehicle.mustang"

param EGO_SPEED = Range(5, 8)
param EGO_BRAKE_DIST = Range(12, 18)
param BLOCKED_VEHICLE_SPEED = 0
param FLOWING_TRAFFIC_SPEED = Range(6, 10)
param VEHICLE_SPACING = Range(8, 12)
param TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoStraightBehavior(trajectory, brake_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior()

behavior StoppedVehicleBehavior():
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

behavior FlowStraightBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego lane and straight maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adjacent right lane for flowing traffic
rightLane = egoInitLane.rightLane
require rightLane is not None
rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightLane.maneuvers))
rightTrajectory = [rightLane, rightManeuver.connectingLane, rightManeuver.endLane]

# Spawn points
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Blocked vehicles in ego lane ahead of ego
blockedPt1 = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.VEHICLE_SPACING
blockedPt2 = new OrientedPoint following egoInitLane.orientation from blockedPt1 for globalParameters.VEHICLE_SPACING
blockedPt3 = new OrientedPoint following egoInitLane.orientation from blockedPt2 for globalParameters.VEHICLE_SPACING

# Flowing traffic in right lane, roughly aligned with ego longitudinally
flowPt1 = new OrientedPoint following rightLane.orientation from egoSpawnPt.offset(right=egoInitLane.width) for Range(-5, 5)
flowPt2 = new OrientedPoint following rightLane.orientation from flowPt1 for globalParameters.VEHICLE_SPACING

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoStraightBehavior(egoTrajectory, globalParameters.EGO_BRAKE_DIST)

# Line of dark adversary vehicles stopped in ego lane
blockedCar1 = new Car at blockedPt1,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_VEHICLE_MODELS),
    with behavior StoppedVehicleBehavior()

blockedCar2 = new Car at blockedPt2,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_VEHICLE_MODELS),
    with behavior StoppedVehicleBehavior()

blockedCar3 = new Car at blockedPt3,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_VEHICLE_MODELS),
    with behavior StoppedVehicleBehavior()

# White vehicle flowing freely in right lane
whiteCar = new Car at flowPt1,
    with regionContainedIn None,
    with blueprint WHITE_VEHICLE_MODEL,
    with behavior FlowStraightBehavior(rightTrajectory, globalParameters.FLOWING_TRAFFIC_SPEED)

# Red vehicle flowing freely in right lane behind white car
redCar = new Car at flowPt2,
    with regionContainedIn None,
    with blueprint RED_VEHICLE_MODEL,
    with behavior FlowStraightBehavior(rightTrajectory, globalParameters.FLOWING_TRAFFIC_SPEED)

# Constraints
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require rightLane is not None

terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST