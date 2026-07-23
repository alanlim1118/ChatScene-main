description = "Ego vehicle merges from an on-ramp onto a highway while an adversary vehicle approaches from behind in the target lane."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED)

param ADV_SPEED = Range(10, 15)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param ADV2_SPEED = Range(10, 15)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adv2 = new Car ahead of ego by Range(60, 80),
	facing ego.heading,
	with blueprint MODEL,
	with behavior Adv2Behavior()

param ADV3_SPEED = Range(10, 15)

behavior Adv3Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV3_SPEED)

adv3 = new Car ahead of ego by Range(30, 50),
	facing ego.heading,
	with blueprint MODEL,
	with behavior Adv3Behavior()

param ADV4_SPEED = Range(10, 15)

behavior Adv4Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV4_SPEED)

adv4 = new Car ahead of ego by Range(100, 120),
	facing ego.heading,
	with blueprint MODEL,
	with behavior Adv4Behavior()

param INIT_DIST = 50
param MIN_PROGRESS = 70
param MERGE_DIST = 20

require (distance to intersection) > INIT_DIST
terminate when (distance from ego to adversary) < MERGE_DIST and (distance to egoSpawnPt) > MIN_PROGRESS