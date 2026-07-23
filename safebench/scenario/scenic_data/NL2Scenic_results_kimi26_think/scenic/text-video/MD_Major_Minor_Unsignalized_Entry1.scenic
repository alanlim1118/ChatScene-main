"""Scenario Description:

Under dark weather conditions at an urban T-junction, the ego vehicle travels straight along the main road while an adversary vehicle follows directly behind it in the same lane. As the ego vehicle proceeds through the junction, a second adversary vehicle emerges from the right arm of the intersection and executes a left turn onto the main road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
param weather = 'Midnight'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 40]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV1_FOLLOW_DIST = [5, 15]
param ADV1_SPEED = VerifaiRange(7, 10)

ADV2_INIT_DIST = [5, 20]
param ADV2_SPEED = VerifaiRange(5, 10)

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

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego travels straight along the main road
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adv1 follows directly behind ego in the same lane
adv1InitLane = egoInitLane
adv1Trajectory = egoTrajectory
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adv2 emerges from the right arm and executes a left turn onto the main road
advOppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
advOppLane = advOppManeuver.startLane

adv2Maneuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.LEFT_TURN and
    m.startLane is not egoInitLane and
    m.startLane is not advOppLane,
    intersection.maneuvers))
adv2InitLane = adv2Maneuver.startLane
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
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV1_SPEED, trajectory=adv1Trajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=adv2Trajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV1_FOLLOW_DIST[0] <= (distance from adversary1 to intersection) - (distance to intersection) <= ADV1_FOLLOW_DIST[1]
require ADV2_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV2_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST