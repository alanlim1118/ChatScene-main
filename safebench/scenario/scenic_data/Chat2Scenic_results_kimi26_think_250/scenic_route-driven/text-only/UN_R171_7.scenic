description = "Ego vehicle follows a lead vehicle in a straight lane and then performs a full lane change into the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToRight

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

terminate when (ego in adjLaneSec)