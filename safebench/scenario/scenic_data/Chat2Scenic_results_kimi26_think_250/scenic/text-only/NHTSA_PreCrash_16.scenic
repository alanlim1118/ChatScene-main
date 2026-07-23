description = "Ego vehicle passes another vehicle on a rural road and encroaches into oncoming traffic."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 30)
param OPT_ADV_DIST = Range(40, 80)

egoLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec.road.forwardLanes is not None and len(laneSec.road.forwardLanes.lanes) == 1 and laneSec.road.backwardLanes is not None and len(laneSec.road.backwardLanes.lanes) == 1:
            egoLaneSecs.append(laneSec)

egoLaneSec = Uniform(*egoLaneSecs)
egoLane = egoLaneSec.lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

oncomingLane = egoLaneSec.road.backwardLanes.lanes[0]
oncomingProjPt = oncomingLane.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following oncomingLane.orientation from oncomingProjPt for -globalParameters.OPT_ADV_DIST

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

param EGO_SPEED = Range(10, 15)
param TRIGGER_DIST = Range(8, 12)
SAFETY_DIST = 15
ONCOMING_LANE_SEC = oncomingLane.sections[0]

behavior EgoBehavior(ego_speed, trigger_dist):
	try:
		do FollowLaneBehavior(target_speed=ego_speed) until withinDistanceToObjsInLane(self, trigger_dist)
		do LaneChangeBehavior(laneSectionToSwitch=ONCOMING_LANE_SEC, is_oppositeTraffic=True, target_speed=ego_speed)
		do FollowLaneBehavior(target_speed=ego_speed, laneToFollow=oncomingLane, is_oppositeTraffic=True)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
		take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.TRIGGER_DIST)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at AdvSpawnPt,
	with behavior AdversaryBehavior()

param LEADING_SPEED = Range(5, 8)

behavior LeadingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEADING_SPEED)

leading = new Car at LeadingSpawnPt,
    with behavior LeadingBehavior()