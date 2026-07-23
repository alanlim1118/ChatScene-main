description = "Ego vehicle approaches an adversarial vehicle reversing backward in the same lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 60)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

behavior ReverseBehavior():
	take SetReverseAction(True)
	while True:
		take SetThrottleAction(0.5)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior ReverseBehavior()

INIT_DIST_MIN = 20
INIT_DIST_MAX = 60
TERM_DIST = 100

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance to egoSpawnPt) > TERM_DIST