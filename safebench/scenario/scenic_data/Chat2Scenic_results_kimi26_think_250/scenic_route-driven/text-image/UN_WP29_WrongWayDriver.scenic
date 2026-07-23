description = "Ego vehicle passes an oncoming vehicle on a straight two-lane road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
selectedRoad = egoLane.road
advLane = selectedRoad.backwardLanes.lanes[0]
advSpawnPt = new OrientedPoint in advLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST_MIN = 20
INIT_DIST_MAX = 60
TERM_DIST = 100

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance from ego to adversary) > TERM_DIST