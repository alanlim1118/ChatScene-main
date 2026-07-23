description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.tesla.model3'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 30)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior()

param ADV_REVERSE_SPEED = Range(2, 4)

behavior ReverseTowardEgoBehavior():
    take SetReverseAction(True)
    while True:
        take SetThrottleAction(globalParameters.ADV_REVERSE_SPEED / 10)

adversary = new Car at advSpawnPt,
    facing toward egoSpawnPt,
    with regionContainedIn None,
    with behavior ReverseTowardEgoBehavior()

param INIT_DIST = Range(10, 30)
param TERM_DIST = 5

require (distance from ego to adversary) >= 10
terminate when (distance from ego to adversary) <= globalParameters.TERM_DIST