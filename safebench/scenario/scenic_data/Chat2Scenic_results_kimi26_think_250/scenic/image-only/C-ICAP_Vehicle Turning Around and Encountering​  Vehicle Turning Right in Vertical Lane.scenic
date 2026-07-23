description = "Blue VUT executing a U-turn and red VUT turning right at a four-way intersection with trajectories intersecting at the estimated collision point."
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

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

INIT_MIN = 50
INIT_MAX = 60
TERM_DIST = 75

require INIT_MIN <= (distance to intersection) <= INIT_MAX
require INIT_MIN <= (distance from adversary to intersection) <= INIT_MAX
terminate when (distance to intersection) > TERM_DIST and (distance from adversary to intersection) > TERM_DIST