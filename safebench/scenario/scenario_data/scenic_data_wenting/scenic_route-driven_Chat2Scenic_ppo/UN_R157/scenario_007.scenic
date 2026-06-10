description = "Ego vehicle approaches sequentially stopped vehicles in its lane and performs a safe stop."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_EGO_TO_MOTO = Range(30, 40)
param OPT_DIST_MOTO_TO_CAR = Range(5, 10)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_EGO_TO_MOTO
carSpawnPt = new OrientedPoint following roadDirection from motoSpawnPt for globalParameters.OPT_DIST_MOTO_TO_CAR

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

behavior WaitBehavior():
    while True:
        wait

moto = new Motorcycle at motoSpawnPt,
    with blueprint MODEL,
    with behavior WaitBehavior()

car = new Car at carSpawnPt,
    with behavior WaitBehavior()

require 30 <= (distance from egoSpawnPt to motoSpawnPt) <= 40
terminate when (distance from ego to egoSpawnPt) > 150
