description = "Ego vehicle loses control on wet roads in a rural area, running off the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

require (distance from egoSpawnPt to intersection) > 50
