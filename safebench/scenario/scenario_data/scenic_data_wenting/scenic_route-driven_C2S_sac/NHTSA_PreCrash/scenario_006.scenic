description = "Ego vehicle travels straight in a rural area at night, under a high speed limit, and departs the road at a non-junction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_PRE_DEPART_DISTANCE = Range(30, 50)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

require (distance from egoSpawnPt to intersection) > 50

terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_PRE_DEPART_DISTANCE + 10) and ego not in network.roads
terminate after 20 seconds
