description = "Ego vehicle drifts into an adjacent vehicle while driving straight on a high-speed urban road."
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
require adjLaneSec is not None
require adjLaneSec.isForward
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint at adjLanePt, facing egoSpawnPt.heading

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST = 50
TERM_DIST = 120

require (distance from egoSpawnPt to intersection) > INIT_DIST
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
