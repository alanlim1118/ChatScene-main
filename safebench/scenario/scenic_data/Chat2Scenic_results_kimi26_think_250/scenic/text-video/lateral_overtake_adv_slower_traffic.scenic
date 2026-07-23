description = "Ego vehicle attempts to overtake a black sedan on a curving rural highway, but both vehicles enter the oncoming lane simultaneously, creating a near-miss conflict with an oncoming scooter."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(8, 12)
param OPT_OVERTAKE_DIST = Range(15, 25)
param OPT_ABORT_DIST = Range(5, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.OPT_OVERTAKE_DIST)
	try:
		do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_ABORT_DIST):
		do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(5, 8)

behavior AdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for 5 seconds
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for 3 seconds
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

black_sedan = new Car ahead of ego by Range(15, 25),
	with color Color(0, 0, 0),
	with behavior AdvBehavior()

param ADV_SPEED = Range(8, 12)

behavior ForwardBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car in egoLaneSec._laneToLeft,
	with behavior ForwardBehavior()

param TRUCK_SPEED = Range(6, 10)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

truck = new Truck ahead of ego by Range(70, 100),
	with behavior TruckBehavior()

param OPT_MOTO_SPEED = Range(10, 16)

behavior MotoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED)

motorcycle = new Motorcycle in egoLaneSec._laneToLeft,
	with behavior MotoBehavior()