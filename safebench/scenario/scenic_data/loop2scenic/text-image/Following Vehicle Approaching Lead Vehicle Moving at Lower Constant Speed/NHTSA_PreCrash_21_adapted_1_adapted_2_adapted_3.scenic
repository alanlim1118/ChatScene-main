description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_FOLLOW_DISTANCE = Range(20, 40)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_FOLLOW_DISTANCE

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = 15

behavior EgoBehavior(speed, safety_dist):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToObjsInLane(self, safety_dist):
		take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

param OPT_ADV_SPEED = Range(3, 5)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at leadSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

param TERM_FOLLOW_DISTANCE = Range(5, 10)
param TERM_TRAVEL_DISTANCE = Range(80, 120)

require (distance from egoSpawnPt to leadSpawnPt) >= 20

terminate when (distance from ego to leadSpawnPt) <= globalParameters.TERM_FOLLOW_DISTANCE or (distance from egoSpawnPt to ego) >= globalParameters.TERM_TRAVEL_DISTANCE