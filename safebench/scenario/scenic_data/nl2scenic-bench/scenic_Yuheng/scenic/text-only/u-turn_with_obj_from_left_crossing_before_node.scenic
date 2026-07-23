"""Scenario Description:

The ego car executes a u-turn at the intersection. Meanwhile, an adversarial object from the left crosses the street before the intersection node.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(2, 5)
param OPT_BRAKE_DISTANCE = Range(8, 12)  # Distance at which the ego vehicle begins to brake
param OPT_ADV_TRIGGER_DIST = Range(40, 60)  # Distance from ego to adv when adv starts crossing

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
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

behavior AdvCrossBehavior():
    """Adversarial agent waits until ego is close enough, then crosses straight through."""
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_TRIGGER_DIST
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

behavior WaitBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego performs a U-turn at the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial agent comes from the left and goes straight, crossing before the intersection
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
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
    with behavior AdvCrossBehavior()

require monitor TrafficLights()
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 15 <= (distance from advSpawnPt to intersection) <= 30