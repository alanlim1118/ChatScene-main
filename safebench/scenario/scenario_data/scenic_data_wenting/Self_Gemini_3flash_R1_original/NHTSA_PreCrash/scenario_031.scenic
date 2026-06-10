description = "Ego vehicle collides with an object while driving straight on a rural road with a high speed limit at night."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
propSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(50, 80)

param EGO_SPEED = Range(22, 28)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior([egoInitLane])

prop = new Debris at propSpawnPt,
	facing propSpawnPt.heading,
	with regionContainedIn None

terminate when ego intersects prop