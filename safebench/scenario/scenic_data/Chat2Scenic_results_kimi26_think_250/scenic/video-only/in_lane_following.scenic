description = "Ego vehicle follows a lead vehicle along a straight road with dashed lane dividers, maintaining consistent distance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 30)

behavior EgoBehavior():
	do FollowLaneBehavior()

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST_MIN = 10
INIT_DIST_MAX = 30
TERM_DIST = 100

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance to egoSpawnPt) > TERM_DIST