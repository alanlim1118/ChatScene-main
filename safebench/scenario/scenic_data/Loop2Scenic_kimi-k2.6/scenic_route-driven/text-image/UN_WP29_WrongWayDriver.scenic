description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearNoon"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToLeft

advSpawnPt = new OrientedPoint in adjLaneSec.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(9, 11)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, is_oppositeTraffic=True)

adversary = new Car at advSpawnPt,
    facing roadDirection,
    with regionContainedIn None,
    with behavior AdvBehavior()

param PASS_DIST = Range(30, 50)

require (distance from ego to advSpawnPt) >= 20
require (distance from adversary to egoSpawnPt) >= 20

terminate when (distance from ego to advSpawnPt) > globalParameters.PASS_DIST and (distance from adversary to egoSpawnPt) > globalParameters.PASS_DIST