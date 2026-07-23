description = "Ego vehicle performs a lane change to bypass a slow  adversary vehicle but cannot return to its original lane because the adversary accelerates. Ego vehicle must then slow down to avoid collision with leading vehicle in new lane."
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

param OPT_ADV_DIST = Range(10, 15)
param OPT_ADV_INIT_SPEED = Range(2, 4)
param ADV_END_SPEED = 2 * Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_INIT_SPEED) \
		until self.lane is not ego.lane
	do FollowLaneBehavior(target_speed=globalParameters.ADV_END_SPEED)

adversary = new Car following roadDirection for globalParameters.OPT_ADV_DIST,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param OPT_ADV_DIST = Range(10, 15)
LEAD_DIST = globalParameters.OPT_ADV_DIST + 10
param OPT_EGO_SPEED = Range(7, 10)
LEAD_SPEED = globalParameters.OPT_EGO_SPEED - 4

behavior LeadBehavior():
	fasterLaneSec = self.laneSection.fasterLane
	do LaneChangeBehavior(
			laneSectionToSwitch=fasterLaneSec,
			target_speed=LEAD_SPEED)
	do FollowLaneBehavior(target_speed=LEAD_SPEED)

lead = new Car following roadDirection for LEAD_DIST,
	with blueprint MODEL,
	with behavior LeadBehavior()


INIT_DIST = 50
TERM_DIST = 100

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require (distance from lead to intersection) > INIT_DIST
require always (adversary.laneSection._fasterLane is not None)
terminate when (distance to egoSpawnPt) > TERM_DIST

param weather = 'ClearNoon'
