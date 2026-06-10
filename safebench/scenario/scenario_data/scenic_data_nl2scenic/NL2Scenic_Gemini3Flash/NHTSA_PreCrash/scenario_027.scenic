"""Scenario Description:

Vehicle stops at a stop sign in a rural area, in daylight, under clear weather conditions, 
at an intersection with a posted speed limit of 35 mph; and proceeds to turn left 
against lateral crossing traffic.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 35 mph is approximately 15.6 m/s
SPEED_LIMIT = 15.6
param EGO_SPEED = 7.0
param ADV_SPEED = 15.0

# Weather/Time settings
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Vehicle Blueprints
EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.tt'

# Distances
EGO_INIT_DIST = [15, 25]
ADV_INIT_DIST = [30, 50]
SAFETY_DIST = 15
STOP_TIME = 3.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    # 1. Drive toward the intersection
    try:
        do FollowLaneBehavior(target_speed=param EGO_SPEED) until (distance to intersection) < 3
    
    # 2. Stop at the stop sign (simulated delay)
    take SetBrakeAction(1.0)
    take SetSpeedAction(0)
    wait for STOP_TIME seconds
    
    # 3. Proceed with the left turn, yielding to lateral traffic if necessary
    try:
        do FollowTrajectoryBehavior(target_speed=param EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
        take SetBrakeAction(1.0)
        take SetSpeedAction(0)

behavior CrossingTrafficBehavior(trajectory):
    # Standard driving behavior for lateral traffic
    do FollowTrajectoryBehavior(target_speed=param ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for an intersection in the rural map
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Define Ego Maneuver (Left Turn)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Define Adversary Maneuver (Lateral Crossing Traffic)
# Find a straight maneuver that conflicts with the ego's left turn
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Spawn Adversary (Crossing Traffic)
adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior CrossingTrafficBehavior(advTrajectory)

# Placement Constraints
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure the adversary is actually lateral (from the side, not head-on)
require abs(relative heading of adversary from ego) > 45 deg

# Termination
terminate when (distance to egoSpawnPt) > 80