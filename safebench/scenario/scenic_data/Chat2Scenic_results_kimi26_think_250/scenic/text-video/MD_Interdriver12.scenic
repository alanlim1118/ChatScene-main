description = "Ego vehicle yields to two vehicles at a T-intersection before merging onto the main road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(3, 6)
param OPT_EGO_YIELD_DIST = Range(8, 12)
param OPT_EGO_MERGE_DIST = Range(6, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until withinDistanceToAnyCars(self, globalParameters.OPT_EGO_YIELD_DIST)
    while withinDistanceToAnyCars(self, globalParameters.OPT_EGO_MERGE_DIST):
        take SetBrakeAction(1)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car ahead of ego by 30,
	with color "red",
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car ahead of ego by 60,
	with color "white",
	with blueprint MODEL,
	with behavior Adv2Behavior()

EGO_INIT_DIST = [15, 25]
ADV1_INIT_DIST = [0, 20]
ADV2_INIT_DIST = [20, 50]
MERGE_TERM_DIST = 80

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV1_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV1_INIT_DIST[1]
require ADV2_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV2_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > MERGE_TERM_DIST