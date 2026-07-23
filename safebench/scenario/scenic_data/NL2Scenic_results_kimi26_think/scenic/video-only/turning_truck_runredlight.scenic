"""Scenario Description:

The ego vehicle is positioned at a large intersection, preparing to execute a left turn, when a large blue and green dump truck enters the frame from the left, speeding through the intersection in violation of a red light. The truck crosses the ego vehicle's path and collides violently with the side of the car, filling the camera view with its cabin and chassis as it passes directly in front. Following the impact, the view stabilizes to show the truck has continued through the intersection, while a yellow construction crane truck is visible driving away in the distance on the cross street.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
DUMP_TRUCK_MODEL = "vehicle.carlamotors.carlacola"
CRANE_TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param OPT_EGO_SPEED = Range(2, 5)
param OPT_ADV_SPEED = Range(15, 20)
param OPT_CRANE_SPEED = Range(4, 8)
param OPT_BRAKE_DISTANCE = Range(5, 8)

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
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
            setClosestTrafficLightStatus(AdvAgent, "red")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

behavior AdvBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

behavior CraneBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_CRANE_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: preparing to turn left at the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: dump truck entering from the left, going straight through the intersection
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Crane truck: visible in the distance on the cross street, driving away
craneSpawnPt = new OrientedPoint in advManeuver.endLane.centerline

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
    with blueprint DUMP_TRUCK_MODEL,
    with color (0.0, 0.5, 0.7),
    with behavior AdvBehavior()

CraneTruck = new Car at craneSpawnPt,
    with heading craneSpawnPt.heading,
    with regionContainedIn None,
    with blueprint CRANE_TRUCK_MODEL,
    with color (1.0, 0.9, 0.0),
    with behavior CraneBehavior()

require monitor TrafficLights()
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 10 <= (distance from egoSpawnPt to intersection) <= 20
require 30 <= (distance from advSpawnPt to intersection) <= 40
require 60 <= (distance from craneSpawnPt to intersection) <= 90