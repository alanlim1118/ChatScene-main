description = "Ego vehicle encounters a stopped vehicle and performs emergency braking or avoidance."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE_TO_STATIONARY = Range(20, 40)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
stationaryCarSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE_TO_STATIONARY

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

behavior AdversaryBehavior():
	while True:
		wait

adversary = new Car at stationaryCarSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

require 20 <= (distance from egoSpawnPt to stationaryCarSpawnPt) <= 40
terminate when (distance from ego to adversary) > 50
terminate after 30 seconds