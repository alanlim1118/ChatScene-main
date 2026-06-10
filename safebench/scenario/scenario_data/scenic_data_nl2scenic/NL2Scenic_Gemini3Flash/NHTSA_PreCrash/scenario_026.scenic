"""Scenario Description:
Vehicle stops at a stop sign in an urban area, in daylight, under clear weather conditions, 
at an intersection with a posted speed limit of 25 mph; and then proceeds against 
lateral crossing traffic.
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

# 25 mph is approximately 11.17 m/s
EGO_SPEED = 11.17
ADV_SPEED = 11.17

# Distance thresholds
EGO_INIT_DIST = [25, 35]
ADV_INIT_DIST = [20, 30]
STOP_THRESHOLD = 4
TERM_DIST = 60

# Weather/Daylight
param weather = 'ClearNoon'

# Models
EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.tt'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoStopAndGo(trajectory):
    """Ego vehicle drives to intersection, stops to simulate a stop sign, and then proceeds."""
    # 1. Approach the intersection
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory) \
            until (distance to intersection) < STOP_THRESHOLD
    
    # 2. Stop at the stop sign/line
    take SetBrakeAction(1.0)
    do wait for 3.0 seconds
    
    # 3. Proceed through the intersection
    do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)

behavior AdversaryBehavior(trajectory):
    """Lateral traffic driving straight through the intersection."""
    do FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select an urban intersection (not necessarily signalized to simulate stop sign environment)
intersection = Uniform(*filter(lambda i: (i.is4Way or i.is3Way) and not i.isSignalized, network.intersections))

# Define Ego's starting point and trajectory
egoInitLane = Uniform(*intersection.incomingLanes)
# We choose a STRAIGHT maneuver for simplicity, or any that crosses lateral traffic
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Define Adversary's starting point and trajectory (Lateral crossing traffic)
# Look for a maneuver that conflicts with Ego's path
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m in egoManeuver.conflictingManeuvers, network.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoStopAndGo(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(advTrajectory)

# Ensure they start at appropriate distances from the intersection
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure the speed limit of the road is appropriate (approx 25mph)
# Note: CARLA speed limits are in m/s. 11.17 m/s is 25mph.
require egoInitLane.speedLimit >= 10 and egoInitLane.speedLimit <= 13

# Termination condition
terminate when (distance to egoSpawnPt) > TERM_DIST