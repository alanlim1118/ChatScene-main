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
WHITE_HATCHBACK = 'vehicle.citroen.c3'
RED_HATCHBACK = 'vehicle.audi.a2'
CITY_BUS = 'vehicle.carlamotors.carlacola'
WHITE_SUV = 'vehicle.audi.etron'
ONCOMING_CAR = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(8, 12)
param LEAD_SPEED = VerifaiRange(8, 12)
param SUV_SPEED = VerifaiRange(6, 10)
param ONCOMING_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(10, 20)
param BRAKE_INTENSITY = VerifaiRange(0.6, 1.0)

TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior BrakeBehavior(brake_val):
    take SetBrakeAction(brake_val)
    wait

behavior FollowAndBrake(trajectory, target, trigger_dist, brake_val):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when (distance to target) < trigger_dist:
        do BrakeBehavior(brake_val)

behavior EgoEvasiveBehavior(straight_trajectory, left_lane_trajectory, lead_vehicle, turning_vehicle):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=straight_trajectory)
    interrupt when (distance to lead_vehicle) < globalParameters.SAFETY_DIST or (distance to turning_vehicle) < globalParameters.SAFETY_DIST:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=left_lane_trajectory)

behavior PedestrianBehavior():
    take SetWalkingSpeedAction(1.0)
    wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Ego center lane (must have left and right adjacent lanes)
egoInitLane = Uniform(*filter(lambda l: l.leftLane is not None and l.rightLane is not None, intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adjacent lanes
leftLane = egoInitLane.leftLane
rightLane = egoInitLane.rightLane

# Left lane trajectory (oncoming traffic lane)
leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftLane.maneuvers))
leftTrajectory = [leftLane, leftManeuver.connectingLane, leftManeuver.endLane]

# Opposite straight lane for the SUV left turn
oppStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oppLane = oppStraightManeuver.startLane

# SUV left-turn trajectory
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, oppLane.maneuvers))
advTrajectory = [oppLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in oppLane.centerline

# Spawn points
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by 15
redSpawnPt = new OrientedPoint in rightLane.centerline
busSpawnPt = new OrientedPoint ahead of leadSpawnPt by 30
oncomingSpawnPt = new OrientedPoint in leftLane.centerline
pedSpawnPt = new OrientedPoint at egoSpawnPt offset by (0 @ 5), facing egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# White SUV turning left across ego's path (create first so lead vehicle can reference it)
suv = new Car at advSpawnPt,
    with blueprint WHITE_SUV,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.SUV_SPEED, trajectory=advTrajectory)

# Lead white hatchback
lead_vehicle = new Car at leadSpawnPt,
    with blueprint WHITE_HATCHBACK,
    with behavior FollowAndBrake(egoTrajectory, suv, 12, globalParameters.BRAKE_INTENSITY)

# Red hatchback in right lane
red_vehicle = new Car at redSpawnPt,
    with blueprint RED_HATCHBACK,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=egoTrajectory)

# City bus further ahead
bus = new Car at busSpawnPt,
    with blueprint CITY_BUS,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=egoTrajectory)

# Oncoming car in the left (oncoming) lane
oncoming = new Car at oncomingSpawnPt,
    with blueprint ONCOMING_CAR,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=leftTrajectory)

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoEvasiveBehavior(egoTrajectory, leftTrajectory, lead_vehicle, suv)

# Pedestrian for urban activity
pedestrian = new Pedestrian at pedSpawnPt,
    with behavior PedestrianBehavior()

require (distance to intersection) > 10
require (distance from suv to intersection) > 10
require (distance from oncoming to intersection) < 40
require (distance from redSpawnPt to egoSpawnPt) < 20
terminate when (distance to egoSpawnPt) > TERM_DIST