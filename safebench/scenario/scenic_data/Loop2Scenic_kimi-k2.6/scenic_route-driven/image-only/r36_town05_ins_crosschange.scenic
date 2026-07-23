description = "Ego vehicle makes a right turn at an intersection and must yield when pedestrian crosses the crosswalk."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and (m.intersection.is4Way or m.intersection.is3Way), egoInitLane.maneuvers))
intersection = egoManeuver.intersection

tempSpawnPt = egoInitLane.centerline[-1]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian right of tempSpawnPt by 5,
	facing ego.heading,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

EGO_INIT_DIST = [20, 25]
TERM_DIST = 50

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST

param weather = 'ClearNoon'
