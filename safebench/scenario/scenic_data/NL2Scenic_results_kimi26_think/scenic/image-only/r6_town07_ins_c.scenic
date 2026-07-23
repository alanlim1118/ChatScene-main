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

param OPT_ADV_DISTANCE = Range(20, 35)  # Proximity within which the adversarial car begins the cut-over
param OPT_BRAKE_DISTANCE = Range(5, 8)  # Distance at which the ego vehicle begins to brake
param OPT_EGO_SPEED = Range(1, 5)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1, 1.2, 1.3)

CONST_PARALLEL_TOL = 5 deg

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
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) 
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)  # Ensure no acceleration during braking
        take SetBrakeAction(1)  # Brake to avoid collision

behavior AdvBehavior():
    # Wait until close enough to ego, then perform cut-over through intersection
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate
   
#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego travels straight through the intersection from the left lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane.right is not None, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoConnectingLane = egoManeuver.connectingLane
egoEndLane = egoManeuver.endLane

# Adversary starts in the adjacent right lane and cuts across into the ego's path
advInitLane = egoInitLane.right
require advInitLane is not None
advTrajectory = [advInitLane, egoConnectingLane, egoEndLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline
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
require -CONST_PARALLEL_TOL < (egoDir - advDir) < CONST_PARALLEL_TOL
require 10 <= (distance from advSpawnPt to intersection) <= 25
require 30 <= (distance from egoSpawnPt to intersection) <= 45