description = "Ego vehicle approaches an adversarial vehicle reversing backward in the same lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 60)

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

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