description = "Ego vehicle collides with an object while driving straight on a rural road with a high speed limit at night."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

param OPT_DISTANCE = Range(50, 80)

propSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

prop = new Debris at propSpawnPt,
	facing propSpawnPt.heading,
	with regionContainedIn None

terminate when ego intersects prop
