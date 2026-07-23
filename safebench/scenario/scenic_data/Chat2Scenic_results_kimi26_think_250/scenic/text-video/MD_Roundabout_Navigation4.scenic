description = "Ego vehicle follows a lead vehicle into a dark multi-lane roundabout, stopping behind it when the lead abruptly yields to three oncoming adversary vehicles."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: not i.is3Way and not i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*egoInitLane.maneuvers)
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

leadSpawnPt = new OrientedPoint in egoInitLane.centerline
egoSpawnPt = new OrientedPoint behind leadSpawnPt by Range(10, 20)

advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint behind advSpawnPt1 by Range(8, 12)
advSpawnPt3 = new OrientedPoint behind advSpawnPt2 by Range(8, 12)

param EGO_SPEED = Range(3, 5)
param EGO_SAFETY_DIST = Range(8, 12)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.EGO_SAFETY_DIST):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(4, 6)
param ADV_SAFE_DIST = Range(8, 12)

behavior AdversaryBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.ADV_SAFE_DIST):
		take SetBrakeAction(1)

adversary = new Car at advSpawnPt1,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV2_SPEED = Range(4, 6)

behavior Adversary2Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=trajectory)

adversary_2 = new Car at advSpawnPt2,
	with blueprint MODEL,
	with behavior Adversary2Behavior(advTrajectory)

param ADV3_SPEED = Range(4, 6)

behavior Adversary3Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV3_SPEED, trajectory=trajectory)

adversary_3 = new Car at advSpawnPt3,
	with blueprint MODEL,
	with behavior Adversary3Behavior(advTrajectory)

param ADV4_SPEED = Range(4, 6)

behavior Adversary4Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV4_SPEED, trajectory=trajectory)

adversary_4 = new Car behind advSpawnPt3 by Range(8, 12),
	with blueprint MODEL,
	with behavior Adversary4Behavior(advTrajectory)

param STOP_DIST = 3

require (distance from ego to leadSpawnPt) >= 10
terminate when (distance from ego to leadSpawnPt) <= STOP_DIST