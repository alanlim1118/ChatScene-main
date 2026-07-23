"""Scenario Description:

In this top-down aerial view of an urban traffic scenario, a red ego vehicle travels straight northward along a multi-lane road, its path indicated by a straight pink trajectory line. Simultaneously, a blue adversarial vehicle approaches from the intersecting street on the right and executes a right turn, following a curved blue trajectory line to merge onto the main road ahead of the ego car. The intersection is equipped with traffic signals and crosswalk markings, bordered by a grassy area with a bus shelter complex to the north and modern apartment buildings to the northeast, all set within a landscape populated by numerous trees and green spaces.

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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(4, 7)
param OPT_ADV_SPEED = Range(3, 6)
param OPT_ADV_DISTANCE = Range(35, 50)  # Distance at which adv starts turning relative to ego
param OPT_BRAKE_DIST = Range(8, 12)

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
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
            setClosestTrafficLightStatus(AdvAgent, "red")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a signalized 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: straight maneuver through intersection (northward)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: right turn from the right incoming lane, merging ahead of ego
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (1.0, 0.0, 0.0),  # Red ego vehicle
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0.0, 0.0, 1.0),  # Blue adversarial vehicle
    with heading advSpawnPt.heading,
    with behavior AdvBehavior()

require monitor TrafficLights()
require CONST_MIN_RIGHT_DEG < (advDir - egoDir) < CONST_MAX_RIGHT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 45
require 10 <= (distance from advSpawnPt to intersection) <= 25