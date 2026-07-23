description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_PED_DISTANCE = Range(15, 30)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

pedSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_GEO_PED_DISTANCE

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)
param SAFETY_DIST = Range(10, 20)

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior()

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