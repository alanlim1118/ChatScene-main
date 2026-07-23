"""Scenario Description:

In a rural environment characterized by a T-junction, a main vertical road intersects with a side road entering from the left, flanked by a field with scattered hay bales and trees on the west and a body of water on the east. A purple vehicle, serving as the ego car, travels straight along the southern lane of the main road towards the intersection. At the junction, a blue adversarial vehicle is turning right from the side road, merging into the lane directly ahead of the ego car, with its path indicated by a cyan trajectory line. Further north in the same lane, a red vehicle drives straight away from the intersection, leaving a pink trajectory trace behind it, presenting a scenario where the ego car must account for the vehicle cutting in from the side.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 15)
param OPT_ADV_SPEED = Range(5, 15)
param OPT_RED_SPEED = Range(5, 15)
param OPT_ADV_DISTANCE = Range(40, 60)
param OPT_BRAKE_DISTANCE = Range(10, 20)

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

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

behavior RedBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_RED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego travels straight on the main road towards the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary turns right from the side road, merging into the lane ahead of ego
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.endLane is egoManeuver.endLane, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Red vehicle further ahead in the same lane, driving away from the intersection
redSpawnPt = new OrientedPoint in egoManeuver.endLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color [0.5, 0, 0.5],
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with color [0, 0, 1],
    with behavior AdvBehavior()

RedVehicle = new Car at redSpawnPt,
    with heading redSpawnPt.heading,
    with regionContainedIn None,
    with color [1, 0, 0],
    with behavior RedBehavior()

# Spatial requirements
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20
require 20 <= (distance from redSpawnPt to intersection) <= 60