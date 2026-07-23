description = "Following vehicle executes a U-turn behind a leading vehicle at a four-way intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(4, 7)

behavior UTurnBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior UTurnBehavior(advTrajectory)

INIT_DIST_MIN = 10
INIT_DIST_MAX = 50
TERM_DIST = 100

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance to egoSpawnPt) > TERM_DIST