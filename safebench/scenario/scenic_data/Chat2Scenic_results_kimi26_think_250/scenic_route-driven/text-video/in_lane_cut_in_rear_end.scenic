description = "White vehicle overtakes red ego and cuts in ahead, forcing severe braking to avoid rear-end collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLane = network.laneAt(egoSpawnPt.position)
egoLaneSec = initLane.sectionAt(egoSpawnPt)
advLaneSec = egoLaneSec.laneToRight
advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(14, 18)
param OPT_CUTIN_DIST = Range(8, 12)

behavior CutInBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego < globalParameters.OPT_CUTIN_DIST)
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with blueprint MODEL,
	with behavior CutInBehavior()
