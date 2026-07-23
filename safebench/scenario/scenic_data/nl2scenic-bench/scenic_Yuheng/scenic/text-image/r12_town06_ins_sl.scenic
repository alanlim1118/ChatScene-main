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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_BRAKE_DIST = Range(8, 12)
param OPT_ADV_START_DISTANCE = Range(30, 50)

CONST_MERGE_ANGLE_MIN = 60 deg
CONST_MERGE_ANGLE_MAX = 120 deg

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoMergeBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait

behavior AdvStraightBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Find an intersection where a right-side merge onto a straight road is possible
intersection = Uniform(*filter(lambda i: i.is3Way or i.is4Way, network.intersections))

# Ego merges from right side road turning left onto main road
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

# Adversary goes straight through the intersection on the main road (conflicting with ego merge)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.endLane is egoManeuver.endLane, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advTrajectoryLine = advInitLane.centerline + advManeuver.connectingLane.centerline + advManeuver.endLane.centerline

# Spawn points
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
    with color (1.0, 0.0, 0.0),
    with behavior EgoMergeBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0.0, 0.0, 1.0),
    with behavior AdvStraightBehavior()

# Ensure the ego is merging from the right side relative to adversary direction
require CONST_MERGE_ANGLE_MIN < (advDir - egoDir) < CONST_MERGE_ANGLE_MAX

# Ensure proper spacing for conflict scenario
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require globalParameters.OPT_ADV_START_DISTANCE <= (distance from advSpawnPt to intersection) <= (globalParameters.OPT_ADV_START_DISTANCE + 20)

# Ensure trajectories actually intersect (merge conflict)
require (distance from egoTrajectoryLine to advTrajectoryLine) < 5