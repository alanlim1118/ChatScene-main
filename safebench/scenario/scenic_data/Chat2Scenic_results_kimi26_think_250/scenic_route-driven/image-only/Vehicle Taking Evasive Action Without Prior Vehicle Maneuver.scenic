description = "Ego vehicle swerves left to avoid an obstacle ahead, crossing into the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_PROP_DISTANCE = Range(20, 30)
param OPT_GEO_ADV_OFFSET = Range(-2, 2)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

propSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_PROP_DISTANCE

leftLaneSec = egoLaneSec._laneToLeft
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_GEO_ADV_OFFSET

ego = new Car at egoSpawnPt,
    with blueprint MODEL

prop = new Debris at propSpawnPt

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with behavior AdversaryBehavior()

require (distance from ego to prop) >= 15
require (distance from ego to adversary) >= 2
TERM_DIST = 50
terminate when (distance to egoSpawnPt) > TERM_DIST