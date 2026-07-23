"""Scenario Description:

Vehicle stops at a stop sign in a rural area, in daylight, under clear weather conditions, at an intersection with a posted speed limit of 35 mph; and proceeds to turn left against lateral crossing traffic.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map in CARLA
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# ENVIRONMENT                   #
#################################

param timeOfDay = 12        # Noon / daylight
param weather = 'Clear'     # Clear weather conditions

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

SPEED_LIMIT_MPH = 35
SPEED_LIMIT_MS = SPEED_LIMIT_MPH * 0.44704  # Convert mph to m/s

EGO_STOP_DURATION = Uniform(2.0, 4.0)       # Time stopped at stop sign
param EGO_SPEED = VerifaiRange(SPEED_LIMIT_MS * 0.8, SPEED_LIMIT_MS)
param EGO_BRAKE = VerifaiRange(0.8, 1.0)

ADV_INIT_DIST = [30, 60]
param ADV_SPEED = VerifaiRange(SPEED_LIMIT_MS * 0.7, SPEED_LIMIT_MS)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoStopAndTurnBehavior(trajectory, stopDuration):
    # Approach and stop at stop sign
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate
    # After stopping for required duration, proceed with left turn
    do WaitBehavior(stopDuration)
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way or 3-way intersection (rural intersections with stop signs)
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Ego performs a left turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary goes straight through the intersection (lateral crossing traffic)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoStopAndTurnBehavior(egoTrajectory, EGO_STOP_DURATION)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure ego starts at reasonable distance before intersection (stop sign location)
require 20 <= (distance from ego to intersection) <= 40
# Ensure adversary is positioned as lateral crossing traffic
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST