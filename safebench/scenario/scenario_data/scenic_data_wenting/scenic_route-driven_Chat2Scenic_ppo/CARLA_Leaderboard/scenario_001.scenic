description = "Ego vehicle loses control on bad road conditions and recovers to its original lane."
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

require egoSpawnPt not in network.intersections
terminate after 25 seconds
