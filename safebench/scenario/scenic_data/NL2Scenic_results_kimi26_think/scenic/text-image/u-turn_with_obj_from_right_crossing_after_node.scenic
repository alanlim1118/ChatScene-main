"""Scenario Description:

A blue ego vehicle is positioned in the bottom lane of a four-way intersection and is executing a U-turn maneuver, indicated by a curved blue arrow looping from the bottom lane towards the right. Simultaneously, a purple adversarial object enters from the left side of the intersection and travels straight across the road to the right, crossing the path of the turning vehicle.

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
param OPT_ADV_SPEED = Range(4, 8)

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego performs a U-turn (left-turn maneuver that returns to the same road)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.startLane.road == m.endLane.road, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary travels straight from the left, crossing the ego's path
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color [0, 0, 255],
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with color [128, 0, 128],
    with behavior AdvBehavior()

require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from advSpawnPt to intersection) <= 40