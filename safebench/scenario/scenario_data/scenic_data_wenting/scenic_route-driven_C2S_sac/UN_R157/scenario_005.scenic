description = "Ego vehicle encounters a stationary motorcycle in the center of a highway curve, requiring emergency braking."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MOTO_MODEL = 'vehicle.yamaha.yzf'
param weather = 'ClearNoon'

param OPT_MOTO_DIST = Range(40, 60)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_DIST

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint 'vehicle.lincoln.mkz_2017'

param ADV_BRAKE = 1.0

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(globalParameters.ADV_BRAKE)

AdvAgent = new Motorcycle at motoSpawnPt,
    with blueprint MOTO_MODEL,
    with behavior StationaryBehavior()

require 40 <= (distance from egoSpawnPt to motoSpawnPt) <= 60
terminate when (distance from ego to AdvAgent) > 70
