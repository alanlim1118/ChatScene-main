"""Scenario Description:

In a top-down view of an urban four-way intersection, the ego vehicle is positioned in a lane behind a blue adversarial car, with a blue trajectory line indicating a planned left turn. The lead blue vehicle is situated directly ahead in the same lane and is executing a left turn onto the perpendicular road, while a green adversarial vehicle approaches from the opposite direction in the oncoming lane. The intersection is flanked by a parking lot containing rows of colorful stacked containers on the left and a green space with trees and a building on the right, with a large structure spanning the road at the top of the scene.

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
BLUE_CAR_MODEL = "vehicle.tesla.model3"
GREEN_CAR_MODEL = "vehicle.audi.tt"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_BLUE_ADV_SPEED = Range(4, 7)
param OPT_GREEN_ADV_SPEED = Range(5, 8)
param OPT_FOLLOW_DISTANCE = Range(12, 18)
param OPT_BRAKE_DISTANCE = Range(6, 10)

CONST_OPPOSITE_DEG = 180 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_OPP_DEG = CONST_OPPOSITE_DEG - CONST_TOL_DEG
CONST_MAX_OPP_DEG = CONST_OPPOSITE_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(BlueAdvAgent, 100):
            setClosestTrafficLightStatus(BlueAdvAgent, "green")
        if withinDistanceToTrafficLight(GreenAdvAgent, 100):
            setClosestTrafficLightStatus(GreenAdvAgent, "green")
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

behavior BlueAdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_BLUE_ADV_SPEED, trajectory=blueTrajectory)
    terminate

behavior GreenAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_GREEN_ADV_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego plans a left turn
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Blue adversarial car is ahead of ego in the same lane, also turning left
blueManeuver = egoManeuver
blueTrajectory = [blueManeuver.startLane, blueManeuver.connectingLane, blueManeuver.endLane]
blueInitLane = blueManeuver.startLane
blueSpawnPt = new OrientedPoint in blueInitLane.centerline

# Green adversarial car approaches from opposite direction (oncoming lane)
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
greenInitLane = greenManeuver.startLane
greenSpawnPt = new OrientedPoint in greenInitLane.centerline

egoDir = egoSpawnPt.heading
greenDir = greenSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

BlueAdvAgent = new Car at blueSpawnPt,
    with heading blueSpawnPt.heading,
    with regionContainedIn None,
    with blueprint BLUE_CAR_MODEL,
    with color (0, 0, 1),
    with behavior BlueAdvBehavior()

GreenAdvAgent = new Car at greenSpawnPt,
    with heading greenSpawnPt.heading,
    with regionContainedIn None,
    with blueprint GREEN_CAR_MODEL,
    with color (0, 1, 0),
    with behavior GreenAdvBehavior()

require monitor TrafficLights()

# Blue car is directly ahead of ego in the same lane
require distance from egoSpawnPt to blueSpawnPt <= globalParameters.OPT_FOLLOW_DISTANCE
require distance from egoSpawnPt to blueSpawnPt >= 8

# Green car is approaching from opposite direction
require CONST_MIN_OPP_DEG < abs(egoDir - greenDir) < CONST_MAX_OPP_DEG

# Reasonable distances to intersection
require 25 <= (distance from egoSpawnPt to intersection) <= 45
require 15 <= (distance from blueSpawnPt to intersection) <= 30
require 30 <= (distance from greenSpawnPt to intersection) <= 60