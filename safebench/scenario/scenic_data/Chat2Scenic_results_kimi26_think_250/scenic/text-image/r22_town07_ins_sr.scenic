description = "Ego vehicle approaches a rural T-junction as an adversarial vehicle turns right from a side road, cutting into the lane ahead."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

adv2Lane = egoManeuver.endLane
adv2SpawnPt = new OrientedPoint in adv2Lane.centerline

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv2Trajectory = [adv2Lane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV1_SPEED = Range(7, 10)

behavior Adv1Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV1_SPEED, trajectory=trajectory)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior Adv1Behavior(adv1Trajectory)

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=trajectory)

adv2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adv2Behavior(adv2Trajectory)

EGO_INIT_DIST = [25, 35]
ADV_INIT_DIST = [15, 25]
TERM_DIST = 100

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST