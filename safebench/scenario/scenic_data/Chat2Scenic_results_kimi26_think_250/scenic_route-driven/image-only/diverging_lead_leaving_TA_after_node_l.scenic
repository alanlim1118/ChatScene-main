description = "Diverging leading vehicle executes a left turn at a four-way intersection with cross and adjacent traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
leadManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
leadInitLane = leadManeuver.startLane
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 15)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leadManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
leadTrajectory = [leadInitLane, leadManeuver.connectingLane, leadManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param OPT_ADV_LEFT_SPEED = Range(6, 9)

behavior LeftTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_LEFT_SPEED, trajectory=trajectory)

left_adversary = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior LeftTurnBehavior(leadTrajectory)