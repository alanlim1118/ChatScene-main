description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(20, 30)

param OPT_ADV_BLOCK_DIST = globalParameters.OPT_LEADING_DIST * 0.3

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToLeft

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_BLOCK_DIST

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with rolename 'hero'

param OPT_ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

barrierSpawnPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.OPT_LEADING_DIST * 0.5,
    facing roadDirection
streetBarrier = new Prop at barrierSpawnPt,
    with blueprint 'static.prop.streetbarrier',
    with allowCollisions True

param INIT_DIST = 50
param TERM_DIST = 100

require (distance to intersection) > 50
require (distance from adversary to intersection) > 50
require (distance from streetBarrier to intersection) > 50

terminate when (distance to egoSpawnPt) > 100
terminate when (ego in adjLaneSec) and (distance from ego to streetBarrier) > (distance from egoSpawnPt to LeadingSpawnPt)