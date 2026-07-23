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

EGO_INIT_DIST = [25, 35]
FRONT_INIT_DIST = [10, 18]
param EGO_SPEED = VerifaiRange(6, 9)
param FRONT_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(6, 9)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 80

#################################
# WEATHER                       #
#################################

param weather = Weather(preset='ClearNoon', sun_altitude=-10, fog_density=0.3, cloudiness=0.9)

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

# Ego goes straight
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Front vehicle in same lane, also going straight
frontSpawnPt = new OrientedPoint in egoInitLane.centerline
frontTrajectory = egoTrajectory

# Left arm adversary: comes from left relative to ego, turns left
leftStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
leftInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        leftStraightManeuver.conflictingManeuvers)
    ).startLane
leftAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, leftInitLane.maneuvers))
leftAdvTrajectory = [leftInitLane, leftAdvManeuver.connectingLane, leftAdvManeuver.endLane]
leftAdvSpawnPt = new OrientedPoint in leftInitLane.centerline

# Right arm adversaries: come from right relative to ego
rightStraightManeuver = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        leftStraightManeuver.conflictingManeuvers)
    )
# Filter out the ego's own reverse maneuver to get true right arm
rightCandidates = filter(lambda m: m.startLane is not egoInitLane, rightStraightManeuver.conflictingManeuvers)
rightInitLane = Uniform(*[m.startLane for m in rightCandidates if m.type is ManeuverType.STRAIGHT])

# Right adversary 1: turns left
rightAdv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, rightInitLane.maneuvers))
rightAdv1Trajectory = [rightInitLane, rightAdv1Maneuver.connectingLane, rightAdv1Maneuver.endLane]
rightAdv1SpawnPt = new OrientedPoint in rightInitLane.centerline

# Right adversary 2: turns right (use same incoming lane or adjacent)
rightAdv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, rightInitLane.maneuvers))
rightAdv2Trajectory = [rightInitLane, rightAdv2Maneuver.connectingLane, rightAdv2Maneuver.endLane]
rightAdv2SpawnPt = new OrientedPoint in rightInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

frontVehicle = new Car at frontSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.FRONT_SPEED, trajectory=frontTrajectory)

leftAdversary = new Car at leftAdvSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=leftAdvTrajectory)

rightAdversary1 = new Car at rightAdv1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightAdv1Trajectory)

rightAdversary2 = new Car at rightAdv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=rightAdv2Trajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require FRONT_INIT_DIST[0] <= (distance from frontVehicle to ego) <= FRONT_INIT_DIST[1]
require 15 <= (distance from leftAdversary to intersection) <= 30
require 15 <= (distance from rightAdversary1 to intersection) <= 30
require 15 <= (distance from rightAdversary2 to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > TERM_DIST