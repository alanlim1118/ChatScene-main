description = "Ego vehicle turns left from the right lane at an intersection while an adversarial object passes straight from the left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None, egoInitLane.maneuvers))
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToLeft
advInitLane = advLaneSec.lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

advSpawnPt = new OrientedPoint in advInitLane.centerline

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with behavior AdversaryBehavior(advTrajectory)