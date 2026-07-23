description = "A vehicle pulls out from a handicapped parking spot into traffic, encountering another vehicle traveling in the same direction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
advSpawnPt = new OrientedPoint on initLane.centerline
egoSpawnPt = new OrientedPoint right of advSpawnPt by Range(2.5, 3.5), facing advSpawnPt.heading

param OPT_EGO_SPEED = Range(5, 8)

behavior EgoBehavior():
    do AccelerateForwardBehavior() for 2 seconds
    do FollowLaneBehavior(laneToFollow=initLane, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()