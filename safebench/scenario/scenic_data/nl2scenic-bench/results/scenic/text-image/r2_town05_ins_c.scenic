"""Scenario Description:

This top-down aerial view depicts an urban traffic scenario on a multi-lane road running horizontally through the frame. A red vehicle, serving as the ego car, is traveling in the lower lane, while a blue vehicle, identified as the adversarial car, occupies the adjacent upper lane slightly ahead, moving in the same direction. A light blue trajectory line extends forward from the blue vehicle, indicating its projected path along the road. The street is flanked by a sidewalk and a modern building complex with landscaping and trees on the upper side, and a large green park area populated with numerous trees on the lower side. Intersections with marked crosswalks are visible at the left and right boundaries of the scene, and street lamps line the sidewalks.

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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(4, 7)
param OPT_ADV_SPEED = Range(4, 7)
param OPT_ADV_LEAD_DISTANCE = Range(10, 25)  # Adversary is slightly ahead of ego
param OPT_BRAKE_DIST = Range(8, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment between two intersections to match the horizontal multi-lane description
intersectionPair = Uniform(*filter(
    lambda pair: len(pair[0].maneuvers) > 0 and len(pair[1].maneuvers) > 0,
    [(i, j) for i in network.intersections for j in network.intersections if i != j]
))
startIntersection = intersectionPair[0]
endIntersection = intersectionPair[1]

# Find a straight maneuver connecting the two intersections
egoManeuver = Uniform(*filter(
    lambda m: m.type is ManeuverType.STRAIGHT and m.endLane.road is startIntersection.roads[0],
    startIntersection.maneuvers
))

egoInitLane = egoManeuver.startLane
# The adjacent upper lane (left lane relative to driving direction)
advInitLane = egoInitLane.leftLane

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectoryLine = advInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

# Spawn ego in the lower (right) lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Spawn adversary in the upper (left) lane, slightly ahead
advSpawnPt = new OrientedPoint following advInitLane.orientation from egoSpawnPt for globalParameters.OPT_ADV_LEAD_DISTANCE,
    offset left by 0

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color "red",
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color "blue",
    with behavior AdvBehavior()

require advInitLane is not None
require 30 <= (distance from egoSpawnPt to startIntersection) <= 60
require distance from ego to AdvAgent >= 5

terminate when distance from ego to endIntersection < 10