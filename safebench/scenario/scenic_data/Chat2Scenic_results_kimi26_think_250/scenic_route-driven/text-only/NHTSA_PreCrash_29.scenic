description = "Ego vehicle goes straight in an urban non-junction area and takes evasive action to avoid an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLane = network.laneAt(egoSpawnPt.position)
propSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(15, 40)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

adversary = new Debris at propSpawnPt