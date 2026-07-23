description = "Ego vehicle goes straight in an urban area and closes in on a slower lead vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DISTANCE = Range(15, 40)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_LEADING_DISTANCE

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(2, 6)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()