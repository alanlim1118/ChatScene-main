description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearSunset"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_ADV1_DIST = Range(30, 50)
param OPT_ADV2_DIST = Range(50, 70)

onRampLanes = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            onRampLanes.append(lane)

egoInitLane = Uniform(*onRampLanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

egoLaneSec = Uniform(*egoInitLane.sections)
require egoLaneSec._laneToRight is not None

highwayLaneSec = egoLaneSec._laneToRight
require highwayLaneSec.isForward

highwayLane = highwayLaneSec.lane

adv1SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV1_DIST
adv2SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV2_DIST

egoTrajectory = [egoInitLane, highwayLane]

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)
param SAFETY_DIST = Range(15, 25)
param MERGE_SPEED = Range(4, 6)

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory) until withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST)
		do FollowLaneBehavior(target_speed=globalParameters.MERGE_SPEED)
		wait for 3 seconds
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior(egoTrajectory)

param ADV2_SPEED = Range(12, 15)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED, laneToFollow=highwayLane)

adv2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adv2Behavior()

param ADV1_SPEED = Range(10, 13)

behavior Adv1Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV1_SPEED, laneToFollow=highwayLane)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior Adv1Behavior()

param TERM_DIST = Range(80, 120)

terminate when (distance from ego to highwayLane.centerline) < 2 and (distance from ego to adv1) > globalParameters.SAFETY_DIST and (distance from ego to adv2) > globalParameters.SAFETY_DIST
terminate after 30 seconds