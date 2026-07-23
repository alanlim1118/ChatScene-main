description = "Ego vehicle travels straight while an adversarial vehicle accelerates and passes on the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToRight
egoLane = egoLaneSec.lane
advLane = advLaneSec.lane
advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior AdvBehavior():
    do AccelerateForwardBehavior()

adversarial = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()