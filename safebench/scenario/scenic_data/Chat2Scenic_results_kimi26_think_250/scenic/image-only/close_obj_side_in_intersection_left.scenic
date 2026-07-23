description = "Ego and adversary vehicles drive side-by-side approaching a four-way intersection with the adversary encroaching into the ego lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for 3 seconds
	do LaneChangeBehavior(self.laneSection._laneToRight, target_speed=globalParameters.ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car left of egoSpawnPt by Range(3, 4),
	with blueprint MODEL,
	with behavior AdversaryBehavior()

EGO_INIT_DIST = [15, 25]
ADV_INIT_DIST = [15, 25]
VEH_DIST = [3, 4]
TERM_DIST = 70

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require VEH_DIST[0] <= (distance from ego to adversary) <= VEH_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST and (distance from adversary to egoSpawnPt) > TERM_DIST