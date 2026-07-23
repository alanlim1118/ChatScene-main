description = "Ego vehicle turning right at a T-intersection while an oncoming adversarial vehicle turns left into the same lane, creating a merging conflict."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
egoInitLane = egoManeuver.startLane
advInitLane = advManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoFollowerSpawnPt = new OrientedPoint behind egoSpawnPt by Range(5, 10)
advFollowerSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)

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

param FOLLOWER_SPEED = Range(7, 10)

follower = new Car at egoFollowerSpawnPt,
    facing egoFollowerSpawnPt.heading,
    with regionContainedIn None,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.FOLLOWER_SPEED, trajectory=egoTrajectory)

param ADV_FOLLOWER_SPEED = Range(7, 10)

advFollower = new Car at advFollowerSpawnPt,
    facing advFollowerSpawnPt.heading,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_FOLLOWER_SPEED, trajectory=advTrajectory)