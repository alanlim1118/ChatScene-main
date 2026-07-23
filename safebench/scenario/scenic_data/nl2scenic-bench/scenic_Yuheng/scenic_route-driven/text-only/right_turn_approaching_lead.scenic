"""Scenario Description:

The ego car initiates a right turn at the intersection. While turning, it approaches a leading adversarial object that is also turning right.

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

param OPT_ADV_SPEED = Range(2, 4)  # Adversary turns right ahead of ego
param OPT_ADV_LEAD_DIST = Range(8, 15)   # Initial distance adv is ahead of ego along trajectory

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary performs the same right turn maneuver as ego
advManeuver = egoManeuver
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane

# Place adversary ahead of ego along the start lane centerline
advSpawnPt = new OrientedPoint following advInitLane.orientation from egoSpawnPt for globalParameters.OPT_ADV_LEAD_DIST

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

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

require 30 <= (distance from egoSpawnPt to intersection) <= 50
require (distance from advSpawnPt to intersection) < (distance from egoSpawnPt to intersection)
require abs(egoDir - advDir) < 10 deg
