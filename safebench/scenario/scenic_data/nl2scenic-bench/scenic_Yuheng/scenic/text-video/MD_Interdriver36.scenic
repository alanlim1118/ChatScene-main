"""Scenario Description:

The ego vehicle travels in the left lane of an urban street under clear weather conditions, approaching a four-way intersection situated beneath a large concrete overpass structure. Intending to make a right turn, the ego vehicle initiates a lane change toward the right lane. However, the target right lane is occupied by a line of three adversary vehicles—specifically a red car followed by two grey vehicles further ahead—traveling straight through the intersection. Consequently, the ego vehicle is forced to manage its spacing and yield to the oncoming traffic in the right lane before it can complete the lane change and execute the turn.

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
RED_CAR_MODEL = 'vehicle.tesla.model3'
GREY_CAR_MODEL = 'vehicle.audi.a2'

EGO_INIT_DIST = [30, 40]
param EGO_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

ADV_INIT_DIST_LEAD = [15, 25]
ADV_SPACING = [8, 12]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 18)
CRASH_DIST = 5
TERM_DIST = 80

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

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection (overpass areas are typically in Town05)
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego starts in the left incoming lane and intends a right turn
egoInitLane = Uniform(*filter(lambda l: len(l.maneuvers) > 1, intersection.incomingLanes))
egoRightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoRightManeuver.connectingLane, egoRightManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries travel straight in the right lane (same direction as ego's initial approach)
# The right lane is identified as a lane with a straight maneuver that shares the same
# incoming road section but is distinct from the ego's left lane
rightLaneCandidates = filter(lambda l: 
    any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers) and l is not egoInitLane,
    intersection.incomingLanes)
advInitLane = Uniform(*rightLaneCandidates)
advStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advStraightManeuver.connectingLane, advStraightManeuver.endLane]

# Spawn points for the three adversary vehicles in the right lane
advLeadSpawnPt = new OrientedPoint in advInitLane.centerline
advMidSpawnPt = new OrientedPoint in advInitLane.centerline
advRearSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: clear conditions
param weather = 'ClearNoon'

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Lead adversary (red car)
adversary_lead = new Car at advLeadSpawnPt,
    with blueprint RED_CAR_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Middle adversary (grey car)
adversary_mid = new Car at advMidSpawnPt,
    with blueprint GREY_CAR_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Rear adversary (grey car)
adversary_rear = new Car at advRearSpawnPt,
    with blueprint GREY_CAR_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Spatial constraints
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST_LEAD[0] <= (distance from adversary_lead to intersection) <= ADV_INIT_DIST_LEAD[1]

# Adversaries are ordered in a line: lead closest to intersection, then mid, then rear
require (distance from adversary_mid to intersection) >= (distance from adversary_lead to intersection) + ADV_SPACING[0]
require (distance from adversary_mid to intersection) <= (distance from adversary_lead to intersection) + ADV_SPACING[1]
require (distance from adversary_rear to intersection) >= (distance from adversary_mid to intersection) + ADV_SPACING[0]
require (distance from adversary_rear to intersection) <= (distance from adversary_mid to intersection) + ADV_SPACING[1]

# Ensure all adversaries are ahead of or near the ego in the right lane relative to intersection
require (distance from adversary_lead to intersection) < (distance from ego to intersection)

terminate when (distance from ego to egoSpawnPt) > TERM_DIST