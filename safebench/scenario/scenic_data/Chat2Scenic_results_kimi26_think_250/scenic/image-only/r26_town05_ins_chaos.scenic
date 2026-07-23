description = "Ego vehicle proceeds straight through a dense urban four-way intersection with multiple adversaries on intersecting trajectories."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior LeftTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior LeftTurnBehavior(advTrajectory)

behavior StraightBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

straightCar = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior StraightBehavior(advTrajectory)

advStraight = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior StraightBehavior(advTrajectory)

behavior AdvStraightBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

advStraight4 = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdvStraightBehavior(advTrajectory)