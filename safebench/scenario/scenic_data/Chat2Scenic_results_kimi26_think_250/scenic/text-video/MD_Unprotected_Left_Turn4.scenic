description = "Ego vehicle executes a left turn at a foggy urban four-way intersection, navigating oncoming and crossing traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

param EGO_SPEED = Range(6, 9)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(3, 5)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

adversary2 = new Car ahead of advSpawnPt by 30,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary3 = new Car ahead of advSpawnPt by Range(40, 50),
	with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)