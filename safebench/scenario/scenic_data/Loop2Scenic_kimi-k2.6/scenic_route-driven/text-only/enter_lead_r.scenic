description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_BLOCK_DIST = Range(20, 30)
param OPT_LEADING_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToRight

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_BLOCK_DIST

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn egoLaneSec,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

adversary = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with behavior AdvBehavior()

param TERM_DIST = 30

require (distance from egoSpawnPt to AdvSpawnPt) >= 20
terminate when (distance from ego to AdvSpawnPt) > (distance from AdvSpawnPt to egoSpawnPt) + globalParameters.OPT_LEADING_DIST + globalParameters.TERM_DIST