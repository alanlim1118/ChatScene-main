"""Scenario Description:

The ego vehicle travels straight on a multi-lane road towards an urban roundabout featuring a central island with a dark circular base and a white spiral structure. As the ego vehicle approaches the junction, a red adversary vehicle travels in the lane to its left, and both cars enter the roundabout at the same time. Meanwhile, other vehicles are visible navigating the circular intersection ahead, with residential buildings lining the left side of the road and a tree-lined pedestrian area on the right.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_OTHER_SPEED = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

behavior AdvBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select the roundabout intersection
intersection = Uniform(*network.intersections)

# Select an incoming lane with a left neighbor for the multi-lane road
egoInitLane = Uniform(*filter(lambda l: l.leftLane is not None, intersection.incomingLanes))
advLane = egoInitLane.leftLane

# Define trajectories through the roundabout
egoManeuver = Uniform(*egoInitLane.maneuvers)
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*advLane.maneuvers)
advTrajectory = [advLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points along the lane centerlines
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.OPT_EGO_SPEED)

# Red adversary vehicle in the left lane
adversary = new Car at advSpawnPt,
    with blueprint EGO_MODEL,
    with color [1, 0, 0],
    with behavior AdvBehavior(advTrajectory, globalParameters.OPT_ADV_SPEED)

# Other vehicles visible near the roundabout
otherLane1 = Uniform(*filter(lambda l: l is not egoInitLane and l is not advLane, intersection.incomingLanes))
otherSpawn1 = new OrientedPoint in otherLane1.centerline
otherCar1 = new Car at otherSpawn1,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_OTHER_SPEED)

otherLane2 = Uniform(*filter(lambda l: l is not egoInitLane and l is not advLane, intersection.incomingLanes))
otherSpawn2 = new OrientedPoint in otherLane2.centerline
otherCar2 = new Car at otherSpawn2,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_OTHER_SPEED)

# Constraints to ensure both ego and adversary enter the roundabout at the same time
require 30 <= (distance from ego to intersection) <= 60
require 30 <= (distance from adversary to intersection) <= 60
require (distance from ego to intersection) - 3 <= (distance from adversary to intersection) <= (distance from ego to intersection) + 3

# Ensure other vehicles are near the intersection
require (distance from otherCar1 to intersection) <= 25
require (distance from otherCar2 to intersection) <= 25

terminate when (distance from ego to egoSpawnPt) > 100