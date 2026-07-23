"""Scenario Description:

This top-down aerial view captures a traffic scenario at a signalized four-way urban intersection situated beneath a large building complex that bridges the northern section of the junction. The ego vehicle, indicated by a pink trajectory line, approaches from the western road, changes lanes to the right, and executes a right turn to proceed northward onto the road passing directly under the building structure. Simultaneously, two adversary vehicles, visible as a blue car and a yellow car (with a red car nearby) aligned with a yellow trajectory line, travel straight through the intersection from the southern approach, occupying the lane that continues under the complex. To the left of the intersection, a parking lot filled with various parked cars and a storage yard with stacked colorful shipping containers are visible, while the bottom right corner features a green park area with trees and a separate building structure.

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
ADV_BLUE_MODEL = "vehicle.tesla.model3"
ADV_YELLOW_MODEL = "vehicle.audi.tt"
ADV_RED_MODEL = "vehicle.ford.mustang"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_ADV_DISTANCE = Range(50, 70)  # Distance at which adversaries begin moving
param OPT_BRAKE_DISTANCE = Range(5, 10)

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
        for adv in [AdvBlue, AdvYellow, AdvRed]:
            if withinDistanceToTrafficLight(adv, 100):
                setClosestTrafficLightStatus(adv, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 3 seconds
        abort
    terminate

behavior AdvStraightBehavior(trigger_distance):
    do WaitBehavior() until (distance from self to ego) < trigger_distance
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a signalized 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: right turn maneuver (west to north)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries: straight maneuver from south (conflicting with ego's right turn)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane

# Spawn points for three adversary vehicles along the same lane
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint ahead of advSpawnPt1 by Range(8, 12)
advSpawnPt3 = new OrientedPoint behind advSpawnPt1 by Range(8, 12)

egoDir = egoSpawnPt.heading
advDir = advSpawnPt1.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvBlue = new Car at advSpawnPt1,
    with heading advDir,
    with regionContainedIn None,
    with blueprint ADV_BLUE_MODEL,
    with behavior AdvStraightBehavior(globalParameters.OPT_ADV_DISTANCE)

AdvYellow = new Car at advSpawnPt2,
    with heading advDir,
    with regionContainedIn None,
    with blueprint ADV_YELLOW_MODEL,
    with behavior AdvStraightBehavior(globalParameters.OPT_ADV_DISTANCE)

AdvRed = new Car at advSpawnPt3,
    with heading advDir,
    with regionContainedIn None,
    with blueprint ADV_RED_MODEL,
    with behavior AdvStraightBehavior(globalParameters.OPT_ADV_DISTANCE)

require monitor TrafficLights()
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 20 <= (distance from advSpawnPt1 to intersection) <= 40