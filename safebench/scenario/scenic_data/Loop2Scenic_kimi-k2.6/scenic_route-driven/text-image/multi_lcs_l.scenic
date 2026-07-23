description = "Ego vehicle performs multiple lane changes to bypass two slow adversary vehicles."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(2, 4)
param OPT_ADV1_DIST = Range(20, 25)

behavior Adversary1Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary_1 = new Car following roadDirection for globalParameters.OPT_ADV1_DIST,
	with blueprint MODEL,
	with behavior Adversary1Behavior()

param OPT_ADV_SPEED = Range(2, 4)
param ADV2_DIST = globalParameters.OPT_ADV1_DIST + Range(15, 20)

behavior Adversary2Behavior():
	rightLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(
		laneSectionToSwitch=rightLaneSec,
		target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary_2 = new Car following roadDirection for globalParameters.ADV2_DIST,
	with blueprint MODEL,
	with behavior Adversary2Behavior()


INIT_DIST = 50
TERM_DIST = globalParameters.ADV2_DIST + 15

require (distance to intersection) > INIT_DIST
require (distance from adversary_1 to intersection) > INIT_DIST
require (distance from adversary_2 to intersection) > INIT_DIST
terminate when (distance to adversary_2) > TERM_DIST

param weather = 'ClearNoon'
