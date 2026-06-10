description = "Ego vehicle travels straight at constant speed with minimal lateral offset before approaching a stationary vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
initLane = network.laneAt(egoSpawnPt.position)

param OPT_DISTANCE = Range(20, 30)

advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL

require (distance from egoSpawnPt to initLane.centerline) <= 0.5
require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate after 10 seconds
