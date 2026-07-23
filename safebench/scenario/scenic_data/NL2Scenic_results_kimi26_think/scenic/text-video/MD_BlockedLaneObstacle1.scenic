"""Scenario Description:

Captured from a top-down aerial perspective, the ego vehicle travels straight along a multi-lane urban street flanked by high-rise buildings under clear weather conditions. As the ego vehicle approaches a four-way intersection, it is forced to wait because a line of dark adversary vehicles is stopped directly ahead in the ego's driving lane, obstructing the path. Meanwhile, in the adjacent right lane, traffic flows freely; a white vehicle and a red vehicle are observed driving straight ahead through the intersection, bypassing the congestion in the left lane. The scene highlights the contrast between the blocked ego lane and the open adjacent lane, with shadows from the buildings and roadside trees cast across the asphalt.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
DARK_ADV_MODELS = ["vehicle.tesla.model3", "vehicle.audi.a2", "vehicle.bmw.grandtourer"]
WHITE_MODEL = "vehicle.seat.leon"
RED_MODEL = "vehicle.nissan.micra"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_FREE_SPEED = Range(8, 12)
param OPT_BRAKE_DIST = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego lane: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adjacent right lane for free-flowing traffic
rightLane = egoInitLane.rightLane
require rightLane is not None

rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightLane.maneuvers))
rightTrajectory = [rightLane, rightManeuver.connectingLane, rightManeuver.endLane]

# Spawn points
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Point of blockage in ego lane, near the intersection
blockagePt = new OrientedPoint in egoInitLane.centerline

# Free-flowing vehicles in the right lane
whiteSpawnPt = new OrientedPoint in rightLane.centerline
redSpawnPt = new OrientedPoint in rightLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Stopped dark adversary vehicles blocking the ego lane
adv1 = new Car at blockagePt,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_ADV_MODELS),
    with behavior WaitBehavior()

adv2 = new Car at new OrientedPoint following egoInitLane.orientation from blockagePt for -4,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_ADV_MODELS),
    with behavior WaitBehavior()

adv3 = new Car at new OrientedPoint following egoInitLane.orientation from blockagePt for -8,
    with regionContainedIn None,
    with blueprint Uniform(*DARK_ADV_MODELS),
    with behavior WaitBehavior()

# Free-flowing white and red vehicles in adjacent right lane
whiteVehicle = new Car at whiteSpawnPt,
    with regionContainedIn None,
    with blueprint WHITE_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_FREE_SPEED, trajectory=rightTrajectory)

redVehicle = new Car at redSpawnPt,
    with regionContainedIn None,
    with blueprint RED_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_FREE_SPEED, trajectory=rightTrajectory)

# Requirements to ensure proper spatial arrangement
require 40 <= (distance to intersection) <= 60
require 10 <= (distance from blockagePt to intersection) <= 20
require 40 <= (distance from whiteSpawnPt to intersection) <= 55
require 40 <= (distance from redSpawnPt to intersection) <= 55
require (distance from whiteSpawnPt to redSpawnPt) >= 5

terminate when (distance from ego to intersection) > 70