description = "A green vehicle approaches a right-hand curve with a stationary motorcycle and blue passenger car as obstacles."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_MOTO_AHEAD = Range(40, 60)
param OPT_CAR_LATERAL = Range(1.5, 2.5)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD

carSpawnPt = new OrientedPoint right of motoSpawnPt by globalParameters.OPT_CAR_LATERAL

ego = new Car at egoSpawnPt,
	with blueprint MODEL

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(1)

moto = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBehavior()

behavior CarStationaryBehavior():
    while True:
        take SetBrakeAction(1)

car = new Car at carSpawnPt,
    with heading carSpawnPt.heading,
    with regionContainedIn None,
    with behavior CarStationaryBehavior()

require 40 <= (distance from ego to moto) <= 60
terminate when (distance from ego to moto) > 90