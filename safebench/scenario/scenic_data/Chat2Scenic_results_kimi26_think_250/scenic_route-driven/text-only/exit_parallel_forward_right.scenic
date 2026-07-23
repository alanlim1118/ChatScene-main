description = "Ego vehicle travels straight within its lane while an adversarial object ahead exits the ego-traffic area to the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(20, 50)

ego = new Car at egoSpawnPt,
    with blueprint MODEL