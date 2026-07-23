"""Scenario Description:

A top-down view of a four-way intersection shows a blue ego vehicle positioned at the bottom lane preparing to execute a left turn, indicated by a blue curved arrow sweeping towards the left road. At the same time, a pink adversarial vehicle approaches from the right lane and performs a right turn, guided by a pink curved arrow pointing towards the upper road, illustrating a simultaneous maneuver where the ego vehicle turns left while an opposing vehicle from the right turns right into the perpendicular lane.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

EGO_INIT_DIST = [20, 30]
ADV_INIT_DIST = [20, 30]

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 80

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego vehicle: left turn from bottom lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: right turn from right lane (relative to ego's approach direction)
# The adversary comes from the right side of the intersection relative to ego
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, 
    filter(lambda m: m.startLane is not egoInitLane and 
                     m.endLane is not egoManeuver.endLane,
           intersection.maneuvers)))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "0,0,255",
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "255,105,180",
    with behavior AdvBehavior(advTrajectory)

# Ensure adversary approaches from the right relative to ego (approximately -90 degrees)
require CONST_MIN_RIGHT_DEG < (advDir - egoDir) < CONST_MAX_RIGHT_DEG

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST