"""Scenario Description:

Under dark nighttime conditions, the ego vehicle approaches and executes a left turn at an urban T-junction. As the ego vehicle enters the intersection from the southern approach, it navigates around cross-traffic on the perpendicular road. An adversary vehicle traveling from the left arm proceeds straight across the intersection towards the right. Concurrently, traffic from the right arm involves multiple agents; two vehicles pass straight through from right to left, while a third vehicle from the right arm executes a left turn. The scene is characterized by low visibility, with vehicle positions and movements primarily defined by their headlights and taillights against the dark asphalt.

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

param OPT_EGO_SPEED = Range(4, 8)
param OPT_ADV_SPEED = Range(5, 9)
param OPT_CROSS_SPEED = Range(5, 9)

EGO_DIST = [25, 35]
ADV_DIST = [25, 35]
RIGHT_DIST = [20, 30]

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, 8):
        take SetBrakeAction(1)
        wait

behavior CrossBehavior(trajectory, target_speed):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*network.intersections)

# Ego: left turn from the stem (southern approach)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Cross-traffic straight maneuvers that conflict with ego's left turn
crossStraightManeuvers = list(filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
require len(crossStraightManeuvers) >= 2

# Adversary: straight from left arm towards the right arm
advManeuver = crossStraightManeuvers[0]
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Right arm: straight from right to left
rightStraightManeuver = crossStraightManeuvers[1]
rightStraightLane = rightStraightManeuver.startLane
rightStraightTrajectory = [rightStraightLane, rightStraightManeuver.connectingLane, rightStraightManeuver.endLane]
rightStraightSpawnPt1 = new OrientedPoint in rightStraightLane.centerline
rightStraightSpawnPt2 = new OrientedPoint at (rightStraightSpawnPt1 offset by -15 @ 0)

# Right arm: left turn onto the stem
rightLeftCandidates = list(filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.startLane is rightStraightLane, intersection.maneuvers))
require len(rightLeftCandidates) >= 1
rightLeftManeuver = Uniform(*rightLeftCandidates)
rightLeftLane = rightLeftManeuver.startLane
rightLeftTrajectory = [rightLeftLane, rightLeftManeuver.connectingLane, rightLeftManeuver.endLane]
rightLeftSpawnPt = new OrientedPoint in rightLeftLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle executing left turn
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Adversary vehicle from left arm proceeding straight
adversary = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior CrossBehavior(advTrajectory, globalParameters.OPT_ADV_SPEED)

# Right arm traffic - two vehicles passing straight through
rightCar1 = new Car at rightStraightSpawnPt1,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior CrossBehavior(rightStraightTrajectory, globalParameters.OPT_CROSS_SPEED)

rightCar2 = new Car at rightStraightSpawnPt2,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior CrossBehavior(rightStraightTrajectory, globalParameters.OPT_CROSS_SPEED)

# Right arm traffic - one vehicle executing left turn
rightCar3 = new Car at rightLeftSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior CrossBehavior(rightLeftTrajectory, globalParameters.OPT_CROSS_SPEED)

#################################
# CONSTRAINTS                   #
#################################

require EGO_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_DIST[1]
require ADV_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_DIST[1]
require RIGHT_DIST[0] <= (distance from rightStraightSpawnPt1 to intersection) <= RIGHT_DIST[1]
require RIGHT_DIST[0] <= (distance from rightLeftSpawnPt to intersection) <= RIGHT_DIST[1]