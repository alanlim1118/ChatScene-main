description = "Diverging leading vehicle executes a left turn at a four-way intersection with cross and adjacent traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
leadManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
leadInitLane = leadManeuver.startLane
leadSpawnPt = new OrientedPoint in leadInitLane.centerline
egoSpawnPt = new OrientedPoint behind leadSpawnPt by Range(10, 15)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leadInitLane.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leadManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
leadTrajectory = [leadInitLane, leadManeuver.connectingLane, leadManeuver.endLane]
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(8, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV_LEFT_SPEED = Range(6, 9)

behavior LeftTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_LEFT_SPEED, trajectory=trajectory)

left_adversary = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior LeftTurnBehavior(leadTrajectory)