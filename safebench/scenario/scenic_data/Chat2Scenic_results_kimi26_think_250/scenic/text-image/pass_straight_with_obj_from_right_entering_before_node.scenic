description = "Ego vehicle travels straight through a four-way intersection while an adversarial agent merges into its path from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 10)
param ADV_OFFSET = Range(2, 4)

adversary = new Car right of egoSpawnPt by globalParameters.ADV_OFFSET,
	facing roadDirection,
	with blueprint MODEL,
	with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [10, 15]
TERM_DIST = 60

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to intersection) > TERM_DIST