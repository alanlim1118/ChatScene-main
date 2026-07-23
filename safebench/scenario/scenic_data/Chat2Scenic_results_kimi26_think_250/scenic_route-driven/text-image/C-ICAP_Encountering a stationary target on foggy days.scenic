description = "Blue VUT on straight multi-lane road approaches stationary shared bicycle offset in lane under limited visibility."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_BIKE_AHEAD_DIST = Range(20, 40)
param OPT_BIKE_RIGHT_OFFSET = Range(1.0, 2.0)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)

bikeCenterPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BIKE_AHEAD_DIST
bikeSpawnPt = new OrientedPoint right of bikeCenterPt by globalParameters.OPT_BIKE_RIGHT_OFFSET,
    facing roadDirection

ego = new Car at egoSpawnPt,
	with blueprint MODEL

behavior BikeBehavior():
	wait

bicycle = new Bicycle at bikeSpawnPt,
	with behavior BikeBehavior()