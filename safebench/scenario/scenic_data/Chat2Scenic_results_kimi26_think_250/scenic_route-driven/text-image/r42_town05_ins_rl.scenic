description = "Ego vehicle turning right at a T-intersection while an oncoming adversarial vehicle turns left into the same lane, creating a merging conflict."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and m.intersection.is3Way, egoInitLane.maneuvers))
advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoFollowerSpawnPt = new OrientedPoint behind egoSpawnPt by Range(5, 10)
advFollowerSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param OPT_FOLLOWER_SPEED = Range(7, 10)

follower = new Car at egoFollowerSpawnPt,
    facing egoFollowerSpawnPt.heading,
    with regionContainedIn None,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_FOLLOWER_SPEED, trajectory=egoTrajectory)

param OPT_ADV_FOLLOWER_SPEED = Range(7, 10)

advFollower = new Car at advFollowerSpawnPt,
    facing advFollowerSpawnPt.heading,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_FOLLOWER_SPEED, trajectory=advTrajectory)