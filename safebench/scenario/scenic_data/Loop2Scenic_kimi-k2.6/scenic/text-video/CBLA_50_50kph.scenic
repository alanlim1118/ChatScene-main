description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 30)

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with rolename 'hero',
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(3, 5)

behavior BicycleForwardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

bicycle = new Bicycle at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior BicycleForwardBehavior()

param EGO_SPEED = Range(13.8, 13.9)

require (distance from egoSpawnPt to advSpawnPt) >= 10
require (distance from egoSpawnPt to advSpawnPt) <= 30

terminate when (ego can see bicycle) and (distance from ego to bicycle) < 2
terminate after 15 seconds