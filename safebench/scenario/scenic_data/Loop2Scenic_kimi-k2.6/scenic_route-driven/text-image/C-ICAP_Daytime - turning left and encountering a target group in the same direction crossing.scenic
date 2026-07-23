description = "Ego vehicle makes a left turn at a 3-way intersection and must suddenly stop to avoid collision when pedestrian crosses the crosswalk."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is3Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

tempManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.reverseManeuvers))
tempInitLane = tempManeuver.startLane
tempSpawnPt = tempInitLane.centerline[-1]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian right of tempSpawnPt by 3,
	facing ego.heading,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

EGO_INIT_DIST = [20, 25]
TERM_DIST = 100

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST

param weather = 'ClearNoon'
