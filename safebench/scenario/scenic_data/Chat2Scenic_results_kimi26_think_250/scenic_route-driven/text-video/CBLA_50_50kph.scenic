description = "Ego vehicle collides with a bicyclist ahead without braking or evasive steering."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
bicycleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(10, 30)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_BICYCLE_SPEED = Range(3, 6)

behavior BicycleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BICYCLE_SPEED)

bicycle = new Bicycle at bicycleSpawnPt,
    facing bicycleSpawnPt.heading,
    with behavior BicycleBehavior()

terminate when (ego intersects bicycle)
