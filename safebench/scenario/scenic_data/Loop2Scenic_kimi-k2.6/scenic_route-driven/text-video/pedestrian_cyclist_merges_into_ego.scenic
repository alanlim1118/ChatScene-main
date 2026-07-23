description = "Ego vehicle travels straight, then avoids a stationary bicycle directly in its path."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

bikeSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(20, 30)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

bicycle = new Bicycle at bikeSpawnPt

require 20 <= (distance from egoSpawnPt to bikeSpawnPt) <= 30
terminate when (distance to bicycle) <= 15 or (distance to egoSpawnPt) > 50