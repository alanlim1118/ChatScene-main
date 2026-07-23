"""Scenario Description:

In a top-down simulation of a four-way intersection, the ego vehicle, depicted as a green box, is positioned centrally and attempts to execute a left turn. The scenario involves navigating amidst cross-traffic, requiring the ego vehicle to yield to approaching adversaries marked as yellow boxes. Specifically, the ego vehicle must wait for a yellow vehicle traveling straight from the opposing arm to clear the intersection, while also monitoring another yellow vehicle approaching from the left arm. The camera view rotates around the intersection, highlighting the spatial relationship between the turning ego vehicle and the conflicting traffic flows from multiple directions.

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

EGO_INIT_DIST = [15, 20]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

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

# Adversary 1: straight from opposing arm
adv1InitLane = Uniform(*intersection.incomingLanes)
adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv1InitLane.maneuvers))
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Ego: left turn from the arm opposite to adv1
egoInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        adv1Maneuver.reverseManeuvers)
    ).startLane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 2: straight from the left arm
# Compute approximate intersection center from incoming lane endpoints
center = sum([lane.centerline.points[-1] for lane in intersection.incomingLanes]) / len(intersection.incomingLanes)

def outwardVector(lane):
    return lane.centerline.points[-1] - center

# Rotate ego's outward vector 90 degrees clockwise to point to the left arm
egoVec = outwardVector(egoInitLane)
leftVec = Vector(egoVec.y, -egoVec.x)

# Select the remaining lane most aligned with the left direction
remainingLanes = [l for l in intersection.incomingLanes if l is not egoInitLane and l is not adv1InitLane]

def alignment(lane):
    v = outwardVector(lane)
    return (v.x * leftVec.x + v.y * leftVec.y) / (abs(v) * abs(leftVec))

adv2InitLane = max(remainingLanes, key=alignment)
adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv2InitLane.maneuvers))
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv1Trajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv2Trajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST