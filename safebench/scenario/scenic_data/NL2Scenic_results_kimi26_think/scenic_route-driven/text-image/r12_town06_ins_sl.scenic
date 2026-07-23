"""Scenario Description:

In a top-down simulation of an urban traffic environment, a red ego vehicle is shown merging from a curved side road on the right onto a main vertical roadway, with its intended path highlighted by a pink trajectory line curving leftward. At the same time, a blue adversarial vehicle travels straight down the main road in the left lane, its path marked by a straight blue line. The scenario depicts a potential conflict as the ego vehicle's merging trajectory intersects with the path of the blue car, indicating that the red vehicle is entering the main road directly into the lane or path of the traffic traveling from the north.

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

param OPT_ADV_SPEED = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego vehicle merges from a side road via a leftward curve onto the main road
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and (m.intersection.is4Way or m.intersection.is3Way), egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary travels straight on the conflicting main road
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from advSpawnPt to intersection) <= 50
