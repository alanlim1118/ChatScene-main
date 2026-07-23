"""Scenario Description:

This top-down aerial view captures a traffic scenario at a rural four-way intersection where the ego vehicle, depicted as a blue car, travels straight through the junction. An adversary vehicle, initially positioned in the adjacent right lane as a red car, performs a cut-over maneuver, merging left to proceed straight directly in front of the ego vehicle's path. The intersection features marked crosswalks and traffic lights, surrounded by a landscape of green trees and grass, with a red barn visible in the bottom left corner and hay bales situated in a field to the bottom right.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.05, 1.15)
param OPT_CUTOVER_DISTANCE = Range(25, 40)  # Distance before intersection when adv begins merge
param OPT_BRAKE_DISTANCE = Range(6, 10)    # Distance at which ego brakes for obstacle ahead

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

behavior AdvCutOverBehavior(cutoverPoint):
    # Drive in right lane until cutover point, then merge left into ego's lane
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to cutoverPoint) < 5
    do ChangeLaneBehavior(direction='left', target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary starts in the right adjacent lane going straight (same direction as ego)
# Find lanes parallel and to the right of ego's initial lane
rightLanes = filter(lambda l: l is not egoInitLane and 
                    abs((l.centerline[0].heading - egoInitLane.centerline[0].heading)) < 10 deg and
                    relativePosition(l.centerline[0], egoInitLane.centerline[0])[1] < 0,
                    intersection.incomingLanes)
advInitLane = Uniform(*rightLanes) if rightLanes else egoInitLane.rightLane

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Define cutover point: a point along the adversary's lane before the intersection
cutoverRegion = advInitLane.centerline
cutoverPoint = new OrientedPoint on cutoverRegion

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (1, 0, 0),
    with behavior AdvCutOverBehavior(cutoverPoint)

require monitor TrafficLights()

# Adversary is to the right of ego (negative lateral offset in ego frame)
require CONST_MIN_RIGHT_DEG < (advDir - egoDir) < CONST_MAX_RIGHT_DEG

# Both vehicles approach the intersection from reasonable distances
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 30 <= (distance from advSpawnPt to intersection) <= 50

# Cutover point is before the intersection entry
require 10 <= (distance from cutoverPoint to intersection) <= 30

# Ensure adversary spawn is laterally offset to the right of ego
require (distance from advSpawnPt to egoInitLane.centerline) > 2