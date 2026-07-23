description = "Ego vehicle performs a lane change into a conflicting space with another merging vehicle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoSection = network.laneSectionAt(egoSpawnPt)
targetSection = egoSection._laneToLeft
advSection = targetSection._laneToLeft

advSpawnPt = new OrientedPoint in advSection.centerline

require 5 < (distance from egoSpawnPt to advSpawnPt) < 15

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(target_speed, target_lane):
	do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
	do FollowLaneBehavior(target_speed=target_speed)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED, targetSection)

require 5 < (distance from egoSpawnPt to advSpawnPt) < 15
terminate when (distance from ego to egoSpawnPt) > 100
terminate after 40 seconds