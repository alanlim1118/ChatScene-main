description = "A green vehicle approaches a right-hand curve with a stationary motorcycle and blue passenger car as obstacles."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_MOTO_AHEAD = Range(40, 60)
param OPT_CAR_LATERAL = Range(1.5, 2.5)

leftLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is None and laneSec._laneToRight is not None:
            leftLaneSecs.append(laneSec)

egoLaneSec = Uniform(*leftLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD

carSpawnPt = new OrientedPoint right of motoSpawnPt by globalParameters.OPT_CAR_LATERAL

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

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