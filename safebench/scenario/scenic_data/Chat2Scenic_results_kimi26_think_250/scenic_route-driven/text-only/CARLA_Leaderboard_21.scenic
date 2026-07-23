description = "Ego-vehicle encounters a pedestrian or bicycle and must perform an emergency brake or an avoidance maneuver."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
lane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following lane.orientation from egoSpawnPt for Range(20, 40)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

adversary = new Pedestrian at advSpawnPt