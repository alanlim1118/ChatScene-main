"""Scenario Description:

From a high-angle aerial view, a blue ego vehicle travels north on a straight urban road towards a T-junction, signaling an intent to turn left. As it approaches the intersection, adversary vehicles, including a grey sedan and a red car, are seen moving from right to left along the perpendicular cross-street. The ego vehicle slows to a stop at the junction line while the cross-traffic also halts, as all vehicles yield to allow a pedestrian to safely cross the road before any further movement occurs.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
GREY_SEDAN_MODEL = "vehicle.tesla.model3"
RED_CAR_MODEL = "vehicle.mustang"

param EGO_SPEED = Range(3, 6)
param ADV_SPEED = Range(3, 6)
param PED_SPEED = Range(0.8, 1.5)

EGO_BRAKE_DIST = Range(8, 12)
ADV_BRAKE_DIST = Range(8, 12)
PED_CROSS_DIST = Range(15, 25)
STOP_DURATION = 6

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior(duration):
    do nothing for duration seconds

behavior EgoLeftTurnBehavior(trajectory, brake_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior(globalParameters.STOP_DURATION)
        abort
    terminate

behavior AdvCrossTrafficBehavior(brake_dist, stop_duration):
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior(stop_duration)
        abort
    terminate

behavior PedestrianCrossBehavior(reference_actor, cross_dist, speed):
    do WalkAlongSidewalkBehavior(speed=speed) until (distance from self to reference_actor <= cross_dist)
    take SetWalkingSpeedAction(0)
    do WaitBehavior(globalParameters.STOP_DURATION + 2)
    take SetWalkingSpeedAction(speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 3-way (T-junction) intersection
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego maneuver: left turn at T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary vehicles on cross-street moving right-to-left relative to ego
# Find conflicting straight maneuvers that cross from ego's right
crossManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers)
advManeuver1 = Uniform(*crossManeuvers)
advInitLane1 = advManeuver1.startLane
advTrajectory1 = [advInitLane1, advManeuver1.connectingLane, advManeuver1.endLane]
advSpawnPt1 = new OrientedPoint in advInitLane1.centerline

# Second adversary on same or adjacent cross lane
advManeuver2 = Uniform(*crossManeuvers)
advInitLane2 = advManeuver2.startLane
advTrajectory2 = [advInitLane2, advManeuver2.connectingLane, advManeuver2.endLane]
advSpawnPt2 = new OrientedPoint in advInitLane2.centerline

# Pedestrian crossing point near the intersection
pedRegion = intersection.region
pedSpawnPt = new OrientedPoint in pedRegion,
    with heading egoInitLane.centerline.end.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoLeftTurnBehavior(egoTrajectory, globalParameters.EGO_BRAKE_DIST)

greySedan = new Car at advSpawnPt1,
    with regionContainedIn None,
    with blueprint GREY_SEDAN_MODEL,
    with color (0.5, 0.5, 0.5),
    with behavior AdvCrossTrafficBehavior(globalParameters.ADV_BRAKE_DIST, globalParameters.STOP_DURATION)

redCar = new Car at advSpawnPt2,
    with regionContainedIn None,
    with blueprint RED_CAR_MODEL,
    with color (1, 0, 0),
    with behavior AdvCrossTrafficBehavior(globalParameters.ADV_BRAKE_DIST, globalParameters.STOP_DURATION)

pedestrian = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianCrossBehavior(ego, globalParameters.PED_CROSS_DIST, globalParameters.PED_SPEED)

# Ensure ego starts at reasonable distance from intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 50

# Ensure adversaries are on the cross-street approaching from right side
require 10 <= (distance from advSpawnPt1 to intersection) <= 30
require 10 <= (distance from advSpawnPt2 to intersection) <= 30

# Ensure adversaries come from roughly the right side of ego (-90 deg relative)
egoHeading = egoSpawnPt.heading
adv1Heading = advSpawnPt1.heading
adv2Heading = advSpawnPt2.heading
require -120 deg < (adv1Heading - egoHeading) < -60 deg
require -120 deg < (adv2Heading - egoHeading) < -60 deg

# Camera setup for high-angle aerial view
record ego from (0, 0, 40) offset by (0, 0, 0),
    with rotation (-70 deg, 0, 0)