description = "Ego vehicle approaches a rural T-junction as an adversarial vehicle turns right from a side road, cutting into the lane ahead."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is3Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

adv2Lane = egoManeuver.endLane
adv2SpawnPt = new OrientedPoint in adv2Lane.centerline

adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv2Trajectory = [adv2Lane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV1_SPEED = Range(7, 10)

behavior Adv1Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV1_SPEED, trajectory=trajectory)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior Adv1Behavior(adv1Trajectory)

param OPT_ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV2_SPEED, trajectory=trajectory)

adv2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adv2Behavior(adv2Trajectory)

EGO_INIT_DIST = [25, 35]
ADV_INIT_DIST = [15, 25]
TERM_DIST = 100

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST