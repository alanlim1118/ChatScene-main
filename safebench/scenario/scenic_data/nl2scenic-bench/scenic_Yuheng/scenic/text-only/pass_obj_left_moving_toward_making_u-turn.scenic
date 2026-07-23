"""Scenario Description:

The ego car is positioned at the intersection. It passes an adversarial object on the left that is moving towards the ego while making a u-turn.

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

param OPT_EGO_SPEED = Range(3, 5)
param OPT_ADV_SPEED = Range(4, 6)
param OPT_BRAKE_DISTANCE = Range(8, 12)
param OPT_ADV_START_DIST = Range(15, 25)

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 25 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            setClosestTrafficLightStatus(AdvAgent, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior AdvUTurnBehavior():
    """Adversary performs a U-turn from the left lane towards the ego's path."""
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a signalized 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary comes from the left and makes a U-turn
# A U-turn from the left relative to ego corresponds to a LEFT_TURN maneuver 
# originating from the opposing direction (conflicting with ego's straight)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane

# Place adversary on its start lane, approaching the intersection from the left side of ego
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_ADV_START_DIST

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvUTurnBehavior()

require monitor TrafficLights()
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 45
require 10 <= (distance from advSpawnPt to intersection) <= 25