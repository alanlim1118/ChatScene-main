"""Scenario Description:

In a top-down simulation of a four-way intersection, the ego vehicle attempts to execute a left turn while navigating amidst cross-traffic. The ego must yield to an adversary traveling straight from the opposing arm and monitor another adversary approaching from the left arm. Both adversaries are marked as yellow boxes, and the camera rotates around the intersection to highlight spatial relationships between the turning ego and conflicting traffic flows.

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
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 80

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

# Ego setup: pick an incoming lane and a left-turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1: opposing arm going straight (conflicts with ego's left turn)
oppStraightManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers)
adv1InitLane = Uniform(*[m.startLane for m in oppStraightManeuvers])
adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv1InitLane.maneuvers))
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adversary 2: left arm relative to ego, going straight through intersection
# The left arm is the incoming lane whose outgoing straight maneuver conflicts with ego's left turn
# We find lanes that are to the left of egoInitLane at this intersection
leftArmLanes = filter(lambda l: 
    l is not egoInitLane and 
    l is not adv1InitLane and
    any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers),
    intersection.incomingLanes)
adv2InitLane = Uniform(*leftArmLanes)
adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, adv2InitLane.maneuvers))
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with color (255, 255, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv1Trajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with color (255, 255, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=adv2Trajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]

# Camera orbits around the intersection center
camera = new Camera at intersection.center offset by (0, 0, 40),
    with heading Uniform(0 deg, 360 deg),
    with pitch -90 deg

terminate when (distance from ego to egoSpawnPt) > TERM_DIST