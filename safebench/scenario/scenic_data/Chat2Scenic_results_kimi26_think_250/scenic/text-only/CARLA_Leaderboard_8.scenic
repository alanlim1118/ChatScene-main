description = "Ego-vehicle reacts to a vehicle merging from a highway on-ramp to avoid collision."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

mergeManeuvers = []
for lane in network.lanes:
	for m in lane.maneuvers:
		startIsRamp = all([sec._laneToLeft is None and sec._laneToRight is None for sec in m.startLane.sections])
		endIsHighway = any([sec._laneToLeft is not None or sec._laneToRight is not None for sec in m.endLane.sections])
		if startIsRamp and endIsHighway and m.startLane is not m.endLane:
			mergeManeuvers.append(m)

advManeuver = Uniform(*mergeManeuvers)
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoInitLane = advManeuver.endLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param OPT_EGO_SPEED = Range(10, 15)
param OPT_SAFETY_DIST = Range(15, 25)
param OPT_BRAKE_DIST = Range(5, 8)
param OPT_BRAKE_FORCE = Range(0.5, 1.0)

behavior EgoBehavior(lane_sec):
	try:
		do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
	interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_SAFETY_DIST):
		do LaneChangeBehavior(laneSectionToSwitch=lane_sec, target_speed=globalParameters.OPT_EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_BRAKE_DIST):
		take SetThrottleAction(0)
		take SetBrakeAction(globalParameters.OPT_BRAKE_FORCE)

ego = new Car at egoSpawnPt,
	with regionContainedIn None,
	with blueprint MODEL,
	with behavior EgoBehavior(egoInitLane.sections[0]._laneToLeft or egoInitLane.sections[0]._laneToRight)

param OPT_ADV_SPEED = Range(10, 14)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()