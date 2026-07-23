"""Scenario Description:

Aborted lead entering from left: A blue ego vehicle travels straight in the center lane of a multi-lane road.
A pink adversary vehicle starts in the adjacent left lane slightly ahead, initiates a merge toward the center lane,
but aborts the maneuver and returns to the left lane. The ego maintains its course throughout.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(8, 12)
param ADV_SPEED = VerifaiRange(8, 12)

EGO_INIT_DIST = [30, 50]
ADV_LEAD_DIST = [10, 20]

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 120

MERGE_DURATION = Range(2.0, 4.0)
ABORT_RETURN_DURATION = Range(2.0, 3.5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(0.8)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior AbortedMergeBehavior(leftLane, centerLane, mergeDur, returnDur, speed):
    # Phase 1: Start merging toward center lane
    mergeTarget = new OrientedPoint in centerLane.centerline
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=[leftLane, centerLane]) for mergeDur
    
    # Phase 2: Abort merge and return to left lane
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=[centerLane, leftLane]) for returnDur
    
    # Phase 3: Continue straight in left lane after aborting
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=[leftLane])

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable multi-lane straight road segment
roadSegment = Uniform(*filter(lambda s: len(s.lanes) >= 3, network.roads))

# Center lane for ego
centerLane = Uniform(*filter(lambda l: l.isForward, roadSegment.lanes))

# Left lane relative to center (adjacent lane to the left)
leftLaneCandidates = filter(lambda l: 
    l.isForward and l is not centerLane and 
    abs(l.centerline.points[0].distanceTo(centerLane.centerline.points[0])) < 6.0,
    roadSegment.lanes)
leftLane = Uniform(*leftLaneCandidates)

# Ego spawn point in center lane
egoSpawnPt = new OrientedPoint in centerLane.centerline

# Adversary spawn point in left lane, slightly ahead of ego
advBasePt = new OrientedPoint in leftLane.centerline
require ADV_LEAD_DIST[0] <= (distance from advBasePt to egoSpawnPt) <= ADV_LEAD_DIST[1]
require (relative heading of advBasePt from egoSpawnPt) < 30 deg

mergeDur = MERGE_DURATION
returnDur = ABORT_RETURN_DURATION

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior([centerLane])

adversary = new Car at advBasePt,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior AbortedMergeBehavior(leftLane, centerLane, mergeDur, returnDur, globalParameters.ADV_SPEED)

require EGO_INIT_DIST[0] <= (distance from ego to roadSegment.start) <= EGO_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST