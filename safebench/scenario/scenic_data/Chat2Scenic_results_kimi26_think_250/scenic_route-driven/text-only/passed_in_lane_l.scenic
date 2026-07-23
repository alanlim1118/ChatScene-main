description = "Ego vehicle drives straight within its lane while an adversarial motorcycle passes from the adjacent left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToLeft
advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_PASS_SPEED = Range(11, 15)

behavior MotorcyclePassBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_PASS_SPEED)

adv = new Motorcycle at advSpawnPt,
    with behavior MotorcyclePassBehavior()