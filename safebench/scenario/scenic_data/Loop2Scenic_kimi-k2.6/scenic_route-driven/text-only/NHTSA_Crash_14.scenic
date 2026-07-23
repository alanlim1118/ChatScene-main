description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearNoon"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight

advSpawnPt = new OrientedPoint in rightLaneSec.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(5, 7)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior([egoLaneSec.lane, rightLaneSec.lane])

require (distance from egoSpawnPt to intersection) > 30
require (distance from advSpawnPt to intersection) > 30
terminate when (ego intersects adversary)