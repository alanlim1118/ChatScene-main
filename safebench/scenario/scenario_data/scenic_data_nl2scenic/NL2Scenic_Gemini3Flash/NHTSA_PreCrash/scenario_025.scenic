"""Scenario Description:

Vehicle is turning left, in daylight, under clear weather conditions, at an intersection 
without traffic controls, with a posted speed limit of 35 mph; and then cuts 
across the path of another vehicle traveling from the opposite direction.

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

# 35 mph is approx 15.6 m/s
TARGET_SPEED = 15.6
EGO_SPEED = 15.6
ADV_SPEED = 12.0

# Distance constants for spawning
EGO_INIT_DIST = [25, 30]
ADV_INIT_DIST = [15, 20]

# Safety parameters
BRAKE_INTENSITY = 1.0
SAFETY_DIST = 15
CRASH_DIST = 5
TERM_DIST = 80

# Weather setup
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    """Ego vehicle drives straight and brakes if the adversary turns in front of it."""
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
        take SetBrakeAction(BRAKE_INTENSITY)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior AdversaryTurnBehavior(trajectory):
    """Adversary performs a left turn across the path of the ego."""
    do FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=trajectory)
    # Continue driving after the turn to clear the intersection
    do FollowLaneBehavior(target_speed=ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for 4-way intersections without traffic lights
intersections = filter(lambda i: i.is4Way and not i.isSignalized, network.intersections)
assert len(intersections) > 0, "No suitable unsignalized 4-way intersection found."
intersec = Uniform(*intersections)

# 1. Define Ego's path (Going Straight)
egoInitLane = Uniform(*intersec.incomingLanes)
egoManeuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, egoInitLane.maneuvers)
egoManeuver = Uniform(*egoManeuvers)
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 2. Define Adversary's path (Coming from opposite side, turning Left)
# Use reverse maneuvers of ego's straight path to find the opposite lane
advInitLane = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers)).startLane
advManeuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, advInitLane.maneuvers)
advManeuver = Uniform(*advManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Calculate Spawn Points
egoSpawnPt = egoInitLane.centerline[-1]
advSpawnPt = advInitLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car following roadDirection from egoSpawnPt for Range(-EGO_INIT_DIST[1], -EGO_INIT_DIST[0]),
    with rolename 'hero',
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car following roadDirection from advSpawnPt for Range(-ADV_INIT_DIST[1], -ADV_INIT_DIST[0]),
    with behavior AdversaryTurnBehavior(advTrajectory)

# Ensure the vehicles are placed correctly relative to the intersection
require EGO_INIT_DIST[0] <= (distance from ego to intersec) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersec) <= ADV_INIT_DIST[1]

# Terminate scenario when ego has traveled far enough
terminate when (distance to egoSpawnPt) > TERM_DIST