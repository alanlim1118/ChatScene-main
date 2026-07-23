"""Scenario Description:

Under dark weather conditions, the ego vehicle travels straight through an urban, multi-lane four-way intersection, following a front vehicle that is also proceeding straight ahead. As the ego vehicle navigates the junction, it encounters a complex mix of traffic where an adversary vehicle on the left arm executes a left turn. Simultaneously, two vehicles on the right arm perform maneuvers, with one turning left and the other turning right, creating a busy intersection environment.

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

EGO_INIT_DIST = [20, 25]
FRONT_INIT_DIST = [5, 15]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [10, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

#################################
# WEATHER                       #
#################################

param weather = {
    'cloudiness': 100.0,
    'precipitation': 0.0,
    'sun_altitude_angle': -30.0,
    'fog_density': 30.0,
    'wetness': 0.0
}

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

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego setup: straight through intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Front vehicle: same lane, ahead of ego, also proceeding straight
frontSpawnPt = new OrientedPoint in egoInitLane.centerline

# Identify opposite lane to exclude it from cross-traffic arms
oppositeStraightManeuver = Uniform(*egoManeuver.reverseManeuvers)
oppositeLane = oppositeStraightManeuver.startLane

# Left arm adversary: left turn conflicting with ego straight
leftAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
leftAdvInitLane = leftAdvManeuver.startLane
leftAdvTrajectory = [leftAdvInitLane, leftAdvManeuver.connectingLane, leftAdvManeuver.endLane]
leftAdvSpawnPt = new OrientedPoint in leftAdvInitLane.centerline

# Right arm lanes: any incoming lane not ego, not left adversary, and not opposite
rightArmLanes = list(filter(lambda l: l is not egoInitLane and l is not leftAdvInitLane and l is not oppositeLane, intersection.incomingLanes))
rightArmLane1 = Uniform(*rightArmLanes)
rightArmLane2 = Uniform(*filter(lambda l: l is not rightArmLane1, rightArmLanes))

# Right arm left-turner
rightLeftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, rightArmLane1.maneuvers))
rightLeftTrajectory = [rightArmLane1, rightLeftManeuver.connectingLane, rightLeftManeuver.endLane]
rightLeftSpawnPt = new OrientedPoint in rightArmLane1.centerline

# Right arm right-turner
rightRightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, rightArmLane2.maneuvers))
rightRightTrajectory = [rightArmLane2, rightRightManeuver.connectingLane, rightRightManeuver.endLane]
rightRightSpawnPt = new OrientedPoint in rightArmLane2.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

frontVehicle = new Car at frontSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

adversary = new Car at leftAdvSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=leftAdvTrajectory)

rightLeftVehicle = new Car at rightLeftSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightLeftTrajectory)

rightRightVehicle = new Car at rightRightSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightRightTrajectory)

# Requirements to ensure proper spawning distances and ordering
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require FRONT_INIT_DIST[0] <= (distance from frontVehicle to intersection) <= FRONT_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from rightLeftVehicle to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from rightRightVehicle to intersection) <= ADV_INIT_DIST[1]
require (distance from frontVehicle to intersection) < (distance from ego to intersection)
require (distance from frontVehicle to ego) > 3

terminate when (distance to egoSpawnPt) > TERM_DIST