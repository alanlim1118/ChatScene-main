description = "Ego and lead vehicles travel at high speed; lead vehicle exits lane to reveal stationary target."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PROP_DIST = Range(20, 30)
param OPT_LEAD_ADDITIONAL_DIST = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

propSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_PROP_DIST
leadSpawnPt = new OrientedPoint following roadDirection from propSpawnPt for globalParameters.OPT_LEAD_ADDITIONAL_DIST

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(20, 25)
param OPT_CUT_OUT_DIST = Range(15, 20)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until withinDistanceToObjsInLane(self, globalParameters.OPT_CUT_OUT_DIST)
	if self.laneSection.laneToLeft:
		targetLane = self.laneSection.laneToLeft
	else:
		targetLane = self.laneSection.laneToRight
	do LaneChangeBehavior(laneSectionToSwitch=targetLane, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

target = new Car at leadSpawnPt,
	with blueprint MODEL

adversary = new NPCCar at propSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()


require 20 <= (distance from egoSpawnPt to propSpawnPt) <= 30
require 15 <= (distance from propSpawnPt to leadSpawnPt) <= 25
terminate when (distance from ego to target) > 70
terminate after 60 seconds
