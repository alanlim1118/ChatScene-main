"""Scenario Description:

Under clear daylight conditions on a multi-lane urban road flanked by residential buildings and leafless trees, the ego vehicle travels forward in the center lane behind a white hatchback while a red hatchback occupies the right lane and a city bus is visible further ahead. As the convoy approaches an intersection, a white SUV from the opposing direction executes a left turn across the ego vehicle's path, causing the lead white hatchback to brake abruptly. Faced with this sudden obstruction and the stopping vehicle ahead, the ego vehicle initiates an emergency evasive maneuver by swerving into the adjacent left lane to avoid a collision, which subsequently results in a side-swipe with an oncoming car in that lane. The scenario highlights a critical chain of events involving unexpected turning traffic, abrupt deceleration by the lead vehicle, and the resulting high-risk lateral evasion amidst pedestrian activity and moderate urban traffic flow.

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
WHITE_HATCHBACK_MODEL = 'vehicle.volkswagen.t2'
RED_HATCHBACK_MODEL = 'vehicle.volkswagen.t2'
BUS_MODEL = 'vehicle.carlamotors.carlacola'
SUV_MODEL = 'vehicle.nissan.patrol'
ONCOMING_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(8, 12)
param LEAD_BRAKE = VerifaiRange(0.8, 1.0)
param ONCOMING_SPEED = VerifaiRange(8, 12)
param TURNER_SPEED = VerifaiRange(4, 7)

LEAD_FOLLOW_DIST = [15, 25]
RIGHT_LANE_OFFSET = [3.5, 4.5]
BUS_AHEAD_DIST = [40, 60]
TURNER_INIT_DIST = [20, 35]
ONCOMING_INIT_DIST = [30, 50]

SAFETY_DIST = 12
CRASH_DIST = 3
TERM_DIST = 100

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.LEAD_BRAKE)
        take SetThrottleAction(0)
        wait

behavior EgoEvasiveBehavior(straightTraj, leftLaneTraj):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=straightTraj)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=leftLaneTraj)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior TurnerBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.TURNER_SPEED, trajectory=trajectory)

behavior OncomingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego starts in center incoming lane going straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoStraightTraj = [egoInitLane, egoStraightManeuver.connectingLane, egoStraightManeuver.endLane]

# Left lane for evasive maneuver (adjacent left of ego init lane)
leftLane = Uniform(*filter(lambda l: l is not egoInitLane and 
                           abs(l.centerline.start.distanceTo(egoInitLane.centerline.start)) < 6,
                           egoInitLane.road.lanes))
leftLaneManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftLane.maneuvers))
leftLaneTraj = [leftLane, leftLaneManeuver.connectingLane, leftLaneManeuver.endLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead white hatchback ahead of ego in same lane
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Uniform(*LEAD_FOLLOW_DIST),
    with heading egoSpawnPt.heading

# Red hatchback in right lane
rightLane = Uniform(*filter(lambda l: l is not egoInitLane and l is not leftLane and
                            abs(l.centerline.start.distanceTo(egoInitLane.centerline.start)) < 6,
                            egoInitLane.road.lanes))
redHatchSpawnPt = new OrientedPoint in rightLane.centerline,
    offset laterally by Uniform(*RIGHT_LANE_OFFSET) relative to egoSpawnPt

# Bus further ahead in ego lane
busSpawnPt = new OrientedPoint ahead of leadSpawnPt by Uniform(*BUS_AHEAD_DIST),
    with heading egoSpawnPt.heading

# White SUV from opposing direction making left turn
opposingLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoStraightManeuver.reverseManeuvers)
    ).startLane
turnerManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, opposingLane.maneuvers))
turnerTraj = [opposingLane, turnerManeuver.connectingLane, turnerManeuver.endLane]
turnerSpawnPt = new OrientedPoint in opposingLane.centerline

# Oncoming car in the left lane (opposing direction)
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftLane.maneuvers))
oncomingReverse = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, oncomingManeuver.reverseManeuvers))
oncomingTraj = [oncomingReverse.startLane, oncomingReverse.connectingLane, oncomingReverse.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingReverse.startLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoEvasiveBehavior(egoStraightTraj, leftLaneTraj)

leadCar = new Car at leadSpawnPt,
    with blueprint WHITE_HATCHBACK_MODEL,
    with color 'white',
    with behavior LeadVehicleBehavior(egoStraightTraj)

redHatch = new Car at redHatchSpawnPt,
    with blueprint RED_HATCHBACK_MODEL,
    with color 'red',
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=[rightLane])

bus = new Car at busSpawnPt,
    with blueprint BUS_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED * 0.8, trajectory=egoStraightTraj)

turner = new Car at turnerSpawnPt,
    with blueprint SUV_MODEL,
    with color 'white',
    with behavior TurnerBehavior(turnerTraj)

oncomingCar = new Car at oncomingSpawnPt,
    with blueprint ONCOMING_MODEL,
    with behavior OncomingBehavior(oncomingTraj)

require TURNER_INIT_DIST[0] <= (distance from turner to intersection) <= TURNER_INIT_DIST[1]
require ONCOMING_INIT_DIST[0] <= (distance from oncomingCar to intersection) <= ONCOMING_INIT_DIST[1]
require LEAD_FOLLOW_DIST[0] <= (distance from ego to leadCar) <= LEAD_FOLLOW_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST