"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, at a non-junction location with a posted speed limit of 35 mph; and takes an evasive action to avoid an obstacle.

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

MODEL = 'vehicle.lincoln.mkz_2017'
OBSTACLE_MODEL = 'static.prop.streetbarrier'

# 35 mph ≈ 15.65 m/s
TARGET_SPEED = 15.65
BRAKE_INTENSITY = 1.0
SAFETY_DISTANCE = 18
CRASH_DISTANCE = 3
TERM_DISTANCE = 80

# Weather: clear daylight
param weather = Weather(preset='ClearNoon')

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        take SetBrakeAction(BRAKE_INTENSITY)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DISTANCE):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction road segment (not part of any intersection)
nonJunctionLanes = filter(lambda l: not any(l in intersec.lanes for intersec in network.intersections), network.lanes)
egoLane = Uniform(*nonJunctionLanes)

# Ensure the lane has a straight maneuver available
straightManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoLane.maneuvers)
require len(list(straightManeuvers)) > 0
egoManeuver = Uniform(*straightManeuvers)
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Spawn ego some distance before the obstacle zone
EGO_SPAWN_DIST = Uniform(-40, -25)
egoSpawnPt = new OrientedPoint following roadDirection from egoLane.centerline[-1] for EGO_SPAWN_DIST

# Place obstacle ahead on the same lane centerline
OBSTACLE_DIST_AHEAD = Uniform(15, 25)
obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for OBSTACLE_DIST_AHEAD

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

obstacle = new Object at obstacleSpawnPt,
    with blueprint OBSTACLE_MODEL,
    with heading egoSpawnPt.heading

# Require that the spawn point is indeed at a non-junction location
require not any(egoSpawnPt in intersec for intersec in network.intersections)

terminate when (distance from ego to egoSpawnPt) > TERM_DISTANCE