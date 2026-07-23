description = "Passing vehicle crosses dashed center line into opposing lane creating head-on conflict with oncoming vehicle."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 20)
param OPT_ONCOMING_DIST = Range(30, 60)

laneSecsWithOpposingLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithOpposingLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithOpposingLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

advLaneSec = egoLaneSec._laneToLeft
oppProjPt = advLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from oppProjPt for globalParameters.OPT_ONCOMING_DIST

param OPT_EGO_SPEED = Range(8, 12)
param OPT_EGO_SAFETY_DIST = Range(10, 15)
param OPT_OVERTAKE_DIST = Range(12, 18)

behavior EgoBehavior(ego_speed, safety_dist, overtake_dist, lane_change_target):
	try:
		do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to LeadingSpawnPt < overtake_dist)
		do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, is_oppositeTraffic=True, target_speed=ego_speed)
		do FollowLaneBehavior(target_speed=ego_speed)
	interrupt when withinDistanceToAnyObjs(self, safety_dist):
		take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(
		globalParameters.OPT_EGO_SPEED,
		globalParameters.OPT_EGO_SAFETY_DIST,
		globalParameters.OPT_OVERTAKE_DIST,
		advLaneSec
	)

param ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior(target_speed):
	do FollowLaneBehavior(target_speed=target_speed)

adversary = new Car at AdvSpawnPt,
	with behavior AdversaryBehavior(globalParameters.ADV_SPEED)

param OPPOSITE_CAR_SPEED = Range(8, 12)

behavior OppositeCarBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

oppositeCar = new Car behind AdvSpawnPt by Range(20, 40),
    facing roadDirection,
    with behavior OppositeCarBehavior(globalParameters.OPPOSITE_CAR_SPEED)

require (distance from egoSpawnPt to intersection) > 0
terminate when (distance from ego to adversary) > 80