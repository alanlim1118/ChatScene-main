"""Scenario Description:

The ego car drives straight forward through the intersection. In doing so, it passes an adversarial object moving parallel to it on the adjacent right lane, fulfilling the "pass object on the right parallel in intersection" scenario.

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

param OPT_EGO_SPEED = Range(4, 6)
param OPT_ADV_SPEED = Range(2, 4)  # Adversary is slower so ego can pass
param OPT_BRAKE_DISTANCE = Range(5, 8)

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 15 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

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

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary goes straight on the adjacent right lane (parallel maneuver)
advManeuver = Uniform(*filter(
    lambda m: m.type is ManeuverType.STRAIGHT and m is not egoManeuver,
    egoManeuver.conflictingManeuvers
))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

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
    with behavior AdvBehavior()

require monitor TrafficLights()

# Ensure adversary is on the right side of ego (parallel, same direction)
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG

# Position both vehicles approaching the intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 45
require 25 <= (distance from advSpawnPt to intersection) <= 40

# Ensure adversary starts slightly ahead or alongside so ego passes it
require (distance from advSpawnPt to intersection) <= (distance from egoSpawnPt to intersection) + 5