description = "A vehicle backing out of a driveway collides with straight-moving traffic."
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

param ADV_SPEED = Range(5, 8)
param ADV_BACK_TIME = Range(1.5, 2.5)
param ADV_TURN_TIME = Range(2.0, 3.0)

behavior AdversaryBehavior():
	take SetReverseAction(True)
	take SetThrottleAction(0.5)
	wait for globalParameters.ADV_BACK_TIME seconds
	take SetReverseAction(False)
	take SetSteerAction(-0.7)
	take SetThrottleAction(0.4)
	wait for globalParameters.ADV_TURN_TIME seconds
	take SetSteerAction(0.0)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car right of egoSpawnPt by Range(5, 9),
	facing -90 deg relative to egoSpawnPt.heading,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST_MIN = 4
INIT_DIST_MAX = 10
TERM_DIST = 80

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance to egoSpawnPt) > TERM_DIST