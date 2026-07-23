description = "Ego vehicle performs a lane changing to evade a leading vehicle, which is moving too slowly."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'

initLane = Uniform(*filter(lambda lane:all([sec._laneToRight is not None for sec in lane.sections]),network.lanes))
egoSpawnPt = new OrientedPoint on initLane.centerline

param EGO_SPEED = Range(9, 12)
BYPASS_DIST = 10

behavior EgoBehavior():
	laneChangeCompleted = False
	try:
		do FollowLaneBehavior(globalParameters.EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST) and not laneChangeCompleted:
		rightLaneSec = self.laneSection._laneToRight
		do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=globalParameters.EGO_SPEED)
		laneChangeCompleted = True

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior()

param LEAD_SPEED = Range(3, 6)
LEAD_INIT_DIST = 10

behavior LeadBehavior():
	do FollowLaneBehavior(globalParameters.LEAD_SPEED)

lead = new Car following roadDirection from ego for LEAD_INIT_DIST,
	with behavior LeadBehavior()

INIT_DIST = 10
TERM_DIST = 50

require (distance from ego to intersection) > INIT_DIST
require (distance from lead to intersection) > INIT_DIST
terminate when (distance to egoSpawnPt) > TERM_DIST

param weather = 'ClearNoon'
