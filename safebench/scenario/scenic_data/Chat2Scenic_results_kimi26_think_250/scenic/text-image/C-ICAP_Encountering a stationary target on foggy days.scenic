description = "Blue VUT on straight multi-lane road approaches stationary shared bicycle offset in lane under limited visibility."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_BIKE_AHEAD_DIST = Range(20, 40)
param OPT_BIKE_RIGHT_OFFSET = Range(1.0, 2.0)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

bikeCenterPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BIKE_AHEAD_DIST
bikeSpawnPt = new OrientedPoint right of bikeCenterPt by globalParameters.OPT_BIKE_RIGHT_OFFSET,
    facing roadDirection

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

behavior BikeBehavior():
	wait

bicycle = new Bicycle at bikeSpawnPt,
	with behavior BikeBehavior()