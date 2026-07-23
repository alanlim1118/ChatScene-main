"""Scenario Description:

Under dark weather conditions at an urban T-junction, the ego vehicle travels straight along the main road while an adversary vehicle follows directly behind it in the same lane. As the ego vehicle proceeds through the junction, a second adversary vehicle emerges from the right arm of the intersection and executes a left turn onto the main road.

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

EGO_INIT_DIST = [20, 40]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV1_INIT_DIST = [8, 15]
param ADV1_SPEED = VerifaiRange(7, 10)

ADV2_INIT_DIST = [10, 25]
param ADV2_SPEED = VerifaiRange(7, 10)

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

# Select a 3-way (T-junction) intersection
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego goes straight through the T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 1 follows directly behind ego in the same lane
adv1SpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary 2 comes from the right arm and turns left onto the main road
# The right arm relative to ego's direction is identified via conflicting maneuvers
# that originate from the right side and perform a left turn into ego's path
rightArmManeuvers = filter(lambda m: 
    m.type is ManeuverType.LEFT_TURN and 
    m.startLane is not egoInitLane and
    m.endLane is egoManeuver.endLane,
    intersection.maneuvers)
adv2Maneuver = Uniform(*rightArmManeuvers)
adv2InitLane = adv2Maneuver.startLane
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Set dark weather conditions
param weather = Weather(darkness=0.8, fog=0.3, wetness=0.5)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV1_SPEED, trajectory=egoTrajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=adv2Trajectory)

# Ego starts at appropriate distance from intersection
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]

# Adversary 1 is behind ego in the same lane
require ADV1_INIT_DIST[0] <= (distance from adversary1 to ego) <= ADV1_INIT_DIST[1]
require heading of adversary1 is heading of ego

# Adversary 2 starts at appropriate distance from intersection on right arm
require ADV2_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV2_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST