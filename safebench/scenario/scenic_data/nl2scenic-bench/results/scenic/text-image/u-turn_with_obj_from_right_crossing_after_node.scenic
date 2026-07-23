"""Scenario Description:

A blue ego vehicle is positioned in the bottom lane of a four-way intersection and is executing a U-turn maneuver, indicated by a curved blue arrow looping from the bottom lane towards the right. Simultaneously, a purple adversarial object enters from the left side of the intersection and travels straight across the road to the right, crossing the path of the turning vehicle.

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

param OPT_EGO_SPEED = Range(3, 5)
param OPT_ADV_SPEED = Range(4, 6)
param OPT_ADV_START_DIST = Range(30, 45)
param OPT_BRAKE_DIST = Range(8, 12)

# U-turn is approximately 180 degrees; adv comes from left (~90 deg relative to ego)
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

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 3 seconds
        abort
    terminate

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way signalized intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego performs a U-turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary goes straight, conflicting with the U-turn
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane

# Spawn adversary at a distance back along its start lane centerline
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_ADV_START_DIST

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color "blue",
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color "purple",
    with behavior AdvBehavior()

require monitor TrafficLights()
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 20 <= (distance from egoSpawnPt to intersection) <= 35
require 25 <= (distance from advSpawnPt to intersection) <= 45