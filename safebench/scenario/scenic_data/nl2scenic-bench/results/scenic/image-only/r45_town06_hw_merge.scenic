"""Scenario Description:

This top-down view shows a highway running vertically between a residential area with houses on the left and a dense forest on the right. The ego vehicle, a green car, travels straight in the rightmost lane, following a red lead vehicle and a yellow car further ahead. Meanwhile, a blue adversary vehicle drives along a curved on-ramp merging from the right, aiming to join the flow of traffic in the lane occupied by the other vehicles.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_MODEL = "vehicle.tesla.model3"
YELLOW_MODEL = "vehicle.audi.tt"
ADV_MODEL = "vehicle.bmw.grandtourer"

param EGO_SPEED = Range(8, 12)
param LEAD_SPEED = Range(7, 11)
param YELLOW_SPEED = Range(7, 11)
param ADV_SPEED = Range(6, 10)

param LEAD_DISTANCE = Range(15, 25)
param YELLOW_DISTANCE = Range(35, 50)
param BRAKE_DIST = Range(8, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior SafeFollowBehavior(target_speed, brake_distance):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_distance):
        take SetBrakeAction(1)

behavior MergeBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to ego) < 5:
        take SetThrottleAction(0)
        take SetBrakeAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a highway segment that has an on-ramp merging from the right
highwaySegments = filter(lambda s: len(s.maneuvers) > 0 and any(m.type is ManeuverType.STRAIGHT for m in s.maneuvers), network.roads)
highwayRoad = Uniform(*highwaySegments)
egoLane = Uniform(*filter(lambda l: l.isRightmost, highwayRoad.lanes))

# Find the on-ramp that merges into this highway road
mergeManeuvers = filter(lambda m: m.type is ManeuverType.MERGE and m.endLane == egoLane, network.maneuvers)
mergeManeuver = Uniform(*mergeManeuvers)
rampLane = mergeManeuver.startLane

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLane.centerline
leadSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.LEAD_DISTANCE
yellowSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.YELLOW_DISTANCE
advSpawnPt = new OrientedPoint in rampLane.centerline

# Define trajectories
egoTrajectory = [egoLane]
advTrajectory = [rampLane, mergeManeuver.connectingLane, mergeManeuver.endLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "green",
    with behavior SafeFollowBehavior(globalParameters.EGO_SPEED, globalParameters.BRAKE_DIST)

leadVehicle = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with color "red",
    with behavior SafeFollowBehavior(globalParameters.LEAD_SPEED, globalParameters.BRAKE_DIST)

yellowVehicle = new Car at yellowSpawnPt,
    with blueprint YELLOW_MODEL,
    with color "yellow",
    with behavior SafeFollowBehavior(globalParameters.YELLOW_SPEED, globalParameters.BRAKE_DIST)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "blue",
    with behavior MergeBehavior(globalParameters.ADV_SPEED)

require 30 <= (distance from egoSpawnPt to mergeManeuver.endLane.centerline.start) <= 80
terminate when (distance from ego to mergeManeuver.endLane.centerline.end) < 10