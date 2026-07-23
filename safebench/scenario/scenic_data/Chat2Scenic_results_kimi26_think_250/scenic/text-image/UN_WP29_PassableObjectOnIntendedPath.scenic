description = "Ego vehicle approaches a small passable obstacle in its lane while driving straight."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
debrisSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 60)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

debris = new Debris at debrisSpawnPt

require 20 <= (distance from egoSpawnPt to debrisSpawnPt) <= 60
terminate when (distance from egoSpawnPt to ego) > ((distance from egoSpawnPt to debrisSpawnPt) + 15)