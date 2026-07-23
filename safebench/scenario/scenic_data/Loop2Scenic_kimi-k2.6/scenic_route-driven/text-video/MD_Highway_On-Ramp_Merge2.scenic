description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearSunset"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_ADV1_DIST = Range(30, 50)
param OPT_ADV2_DIST = Range(50, 70)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
require egoLaneSec._laneToRight is not None

highwayLaneSec = egoLaneSec._laneToRight
require highwayLaneSec.isForward

highwayLane = highwayLaneSec.lane

adv1SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV1_DIST
adv2SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV2_DIST

param SAFETY_DIST = Range(15, 25)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV2_SPEED = Range(12, 15)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV2_SPEED, laneToFollow=highwayLane)

adv2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adv2Behavior()

param OPT_ADV1_SPEED = Range(10, 13)

behavior Adv1Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV1_SPEED, laneToFollow=highwayLane)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior Adv1Behavior()

param OPT_TERM_DIST = Range(80, 120)

terminate when (distance from ego to highwayLane.centerline) < 2 and (distance from ego to adv1) > globalParameters.SAFETY_DIST and (distance from ego to adv2) > globalParameters.SAFETY_DIST
terminate after 30 seconds