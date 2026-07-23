description = "Ego vehicle collides with a bicyclist ahead without braking or evasive steering."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
bicycleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(10, 30)

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param BICYCLE_SPEED = Range(3, 6)

behavior BicycleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.BICYCLE_SPEED)

bicycle = new Bicycle at bicycleSpawnPt,
    facing bicycleSpawnPt.heading,
    with behavior BicycleBehavior()

terminate when (ego intersects bicycle)