description = "Ego vehicle exits a parallel parking bay into traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_START_DIST = Range(10, 20) * -1

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
lanePt = new OrientedPoint left of egoSpawnPt by 3, facing egoSpawnPt.heading
advSpawnPt = new OrientedPoint following roadDirection from lanePt for globalParameters.OPT_ADV_START_DIST

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()