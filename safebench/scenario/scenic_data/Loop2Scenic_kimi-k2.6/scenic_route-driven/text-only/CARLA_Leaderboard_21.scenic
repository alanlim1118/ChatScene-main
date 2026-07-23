description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_PED_DISTANCE = Range(15, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)

pedSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_GEO_PED_DISTANCE

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_PED_SPEED = Range(0.5, 1.5)

behavior PedestrianObstacleBehavior():
    do WalkForwardBehavior(globalParameters.OPT_PED_SPEED)

ped = new Pedestrian ahead of egoSpawnPt by globalParameters.OPT_GEO_PED_DISTANCE,
    facing toward egoSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianObstacleBehavior()

param TERM_BUFFER = Range(25, 35)

require (distance from egoSpawnPt to pedSpawnPt) >= 15

terminate when (distance from ego to pedSpawnPt) > globalParameters.TERM_BUFFER and (distance from ego to egoSpawnPt) > 30