description = "Ego vehicle detects a stationary vehicle with lateral offset on a curved road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(15, 25)
param OPT_OFFSET = Range(0.8, 1.5) * Uniform(-1, 1)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

targetPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST
advSpawnPt = new OrientedPoint left of targetPt by globalParameters.OPT_OFFSET

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_ADV_DIST + 15)
