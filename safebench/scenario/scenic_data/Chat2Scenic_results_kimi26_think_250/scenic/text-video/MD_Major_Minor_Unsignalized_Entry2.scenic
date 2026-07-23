description = "Ego vehicle executes a left turn at an urban T-junction while navigating around multiple cross-traffic vehicles."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(4, 7)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV2_SPEED = Range(5, 8)

behavior Adversary2Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=trajectory)

adversary2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adversary2Behavior(adv2Trajectory)

param ADV3_SPEED = Range(5, 8)

behavior Adversary3Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV3_SPEED, trajectory=trajectory)

adversary3 = new Car at adv3SpawnPt,
	with blueprint MODEL,
	with behavior Adversary3Behavior(adv3Trajectory)

param ADV4_SPEED = Range(5, 8)

behavior Adversary4Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV4_SPEED, trajectory=trajectory)

adversary4 = new Car at adv4SpawnPt,
	with blueprint MODEL,
	with behavior Adversary4Behavior(adv4Trajectory)