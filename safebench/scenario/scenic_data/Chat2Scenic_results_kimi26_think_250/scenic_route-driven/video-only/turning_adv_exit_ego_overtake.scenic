description = "Ego vehicle overtakes a slowing grey sedan turning left across the lane, resulting in a side-impact collision before slowing and passing pedestrians."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

param OPT_ADV_INIT_DIST = Range(15, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_ADV_INIT_DIST
pedRefPt = new OrientedPoint following egoLane.orientation from advSpawnPt for Range(20, 30)
pedSpawnPt = new OrientedPoint right of pedRefPt by Range(3, 5)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_PED_SPEED = Range(0.8, 1.4)

ped = new Pedestrian at pedSpawnPt,
    with behavior WalkForwardBehavior(globalParameters.OPT_PED_SPEED)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance from ego to pedSpawnPt) > 70
terminate after 30 seconds