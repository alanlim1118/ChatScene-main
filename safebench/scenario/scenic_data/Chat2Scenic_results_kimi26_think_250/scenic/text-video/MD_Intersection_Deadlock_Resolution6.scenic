description = "Ego vehicle travels straight north through a four-way urban intersection while an oncoming vehicle turns right and two vehicles enter from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car with blueprint MODEL, with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(trajectory=advTrajectory)

param ADV2_SPEED = Range(8, 12)

behavior Adv2Behavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adversary2 = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior Adv2Behavior(globalParameters.ADV2_SPEED)

param ADV3_SPEED = Range(7, 10)

behavior Adv3Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV3_SPEED, trajectory=trajectory)

adversary3 = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior Adv3Behavior(advTrajectory)

EGO_INIT_DIST = [15, 25]
ADV_INIT_DIST = [10, 20]
TERM_DIST = 80

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary3 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST