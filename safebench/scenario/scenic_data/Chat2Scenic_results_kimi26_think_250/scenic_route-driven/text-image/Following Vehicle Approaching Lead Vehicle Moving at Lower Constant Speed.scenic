description = "Ego vehicle closes in on a slower lead vehicle while traveling straight along an urban lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.carlamotors.firetruck'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 20)
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
lane = network.laneAt(egoSpawnPt.position)
leadSpawnPt = new OrientedPoint following lane.orientation from egoSpawnPt for globalParameters.OPT_LEADING_DIST

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(3, 5)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

TERM_DIST = 100

require 10 <= (distance from ego to adversary) <= 20
terminate when (distance to egoSpawnPt) > TERM_DIST