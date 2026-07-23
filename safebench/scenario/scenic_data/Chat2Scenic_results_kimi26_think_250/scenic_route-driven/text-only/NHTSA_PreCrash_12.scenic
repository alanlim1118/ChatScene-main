description = "Vehicle backs up in an urban driveway or alley and collides with another vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(-10, -5)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

stationary_adv = new Car at advSpawnPt,
	with blueprint MODEL