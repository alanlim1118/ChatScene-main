description = "White vehicle overtakes red ego and cuts in ahead, forcing severe braking to avoid rear-end collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda lane:
	all([sec._laneToRight is not None for sec in lane.sections]),
	network.lanes))
egoSpawnPt = new OrientedPoint in initLane.centerline
egoLaneSec = initLane.sectionAt(egoSpawnPt)
advLaneSec = egoLaneSec.laneToRight
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(8, 12)
param EGO_BRAKE = Range(0.8, 1.0)
param SAFETY_DIST = Range(10, 15)

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(14, 18)
param CUTIN_DIST = Range(8, 12)

behavior CutInBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until (distance from self to ego < globalParameters.CUTIN_DIST)
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with blueprint MODEL,
	with behavior CutInBehavior()