description = "Ego vehicle encounters a pedestrian crossing the lane, requiring deceleration to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE_AHEAD = Range(20, 35)
param OPT_SIDE_OFFSET = Range(4, 6)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLane = network.laneAt(egoSpawnPt.position)

CrossingPoint = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE_AHEAD
pedSpawnPt = new OrientedPoint right of CrossingPoint by globalParameters.OPT_SIDE_OFFSET, facing toward CrossingPoint

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_PED_MIN_SPEED = Range(1.0, 1.5)
param OPT_PED_THRESHOLD = Range(20, 30)

behavior PedestrianBehavior(min_speed, threshold):
	do CrossingBehavior(ego, min_speed, threshold)

pedestrian = new Pedestrian at pedSpawnPt,
	with regionContainedIn None,
	with behavior PedestrianBehavior(globalParameters.OPT_PED_MIN_SPEED, globalParameters.OPT_PED_THRESHOLD)

require ego can see pedestrian
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_DISTANCE_AHEAD + 15)