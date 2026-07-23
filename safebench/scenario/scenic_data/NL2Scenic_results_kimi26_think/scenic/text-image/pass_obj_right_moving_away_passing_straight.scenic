"""Scenario Description:

In a top-down view of a four-way intersection featuring gray roads and white dashed lane markings, a blue ego vehicle is stationary in the center lane facing north. A pink adversarial vehicle is driving straight across the intersection from left to right, perpendicular to the ego vehicle's path. The pink car is positioned to the right of the blue car, indicating it is in the process of crossing, while a dashed red line to the left illustrates its trajectory across the junction.

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
ADV_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_ADV_SPEED = Range(5, 15)

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: stationary in the center lane facing north (straight through the intersection)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: driving straight across from left to right, perpendicular to ego's path
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color [0, 0, 1],  # Blue
    with behavior StationaryBehavior()

AdvAgent = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color [1, 0.4, 0.7],  # Pink
    with heading advSpawnPt.heading,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)

# Ensure perpendicular approach (adversary from the left of the ego)
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
# Keep both agents near the intersection
require 0 <= (distance from egoSpawnPt to intersection) <= 5
require 0 <= (distance from advSpawnPt to intersection) <= 10