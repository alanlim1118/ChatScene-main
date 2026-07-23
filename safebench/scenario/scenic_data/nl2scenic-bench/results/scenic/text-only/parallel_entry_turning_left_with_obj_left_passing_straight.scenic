"""Scenario Description:

The ego car executes a left turn at the intersection from the right lane. Simultaneously, an adversarial object entering parallel from the left lane proceeds straight ahead, illustrating the scenario of a parallel entry turning left with an object from the left passing straight.

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

param OPT_EGO_SPEED = Range(3, 5)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_EGO_BRAKE_DIST = Range(8, 12)
param OPT_ADV_START_OFFSET = Range(-5, 5)  # Longitudinal offset of adv relative to ego at spawn

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_EGO_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior AdvStraightBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection that supports a left turn from a lane that has a left neighbor
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))

# Ensure the ego start lane has a left lane for the adversarial vehicle
require egoManeuver.startLane._laneToLeft is not None
require egoManeuver.startLane._laneToLeft.isForward

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial maneuver: straight through the same intersection from the left lane
advLane = egoManeuver.startLane._laneToLeft
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane is advLane, intersection.maneuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Place adversarial vehicle in the left lane, roughly parallel to ego
advCenterlinePoint = advLane.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLane.orientation from advCenterlinePoint for globalParameters.OPT_ADV_START_OFFSET

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
    with behavior AdvStraightBehavior()

require 30 <= (distance from egoSpawnPt to intersection) <= 50