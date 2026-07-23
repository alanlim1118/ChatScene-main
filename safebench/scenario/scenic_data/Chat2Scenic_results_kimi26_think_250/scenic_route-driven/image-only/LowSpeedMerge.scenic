description = "Ego vehicle changes lanes to merge into a gap between two vehicles in the adjacent lane in preparation for a turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_AHEAD_DIST = Range(10, 30)
param OPT_BEHIND_DIST = Range(10, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToRight
mergePos = adjLaneSec.centerline.project(egoSpawnPt.position)
advAheadSpawnPt = new OrientedPoint following roadDirection from mergePos for globalParameters.OPT_AHEAD_DIST
advBehindSpawnPt = new OrientedPoint following roadDirection from mergePos for -globalParameters.OPT_BEHIND_DIST

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advAheadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

behavior AdversaryBehindBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversaryBehind = new Car at advBehindSpawnPt,
	with behavior AdversaryBehindBehavior()

param OPT_TERM_DIST = Range(70, 100)

require (distance from ego to adversary) >= 10
require (distance from ego to adversaryBehind) >= 10

terminate when (distance to egoSpawnPt) > globalParameters.OPT_TERM_DIST