"""Scenario Description:

The ego car executes a left turn at the intersection. Meanwhile, an adversarial object entering from the right makes a u-turn within the intersection.

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
param OPT_ADV_SPEED = Range(3, 5)
param OPT_BRAKE_DIST = Range(5, 8)
param OPT_ADV_START_DIST = Range(5, 10)

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent) < globalParameters.OPT_BRAKE_DIST:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior UTurnBehavior():
    # Drive up to the intersection from the right
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)
    # Perform a u-turn inside the intersection by steering left and driving slowly
    for _ in range(80):
        take SetThrottleAction(0.25)
        take SetSteerAction(1.0)
    take SetThrottleAction(0)
    take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and (i.isSignalized == False), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary enters from the right and approaches the intersection
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_ADV_START_DIST

# Trajectory limited to the approach lane so the u-turn happens inside the intersection
advTrajectory = [advInitLane]

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
    with behavior UTurnBehavior()

require 40 <= (distance from egoSpawnPt to intersection) <= 50
require 10 <= (distance from advSpawnPt to intersection) <= 20
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG