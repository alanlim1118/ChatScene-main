description = "Pedestrian struck by vehicle while crossing multi-lane road; driver failed to see pedestrian."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_distAhead = Range(20, 30)
param OPT_distSideways = Range(5, 10)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoSection = network.laneSectionAt(egoSpawnPt)

intermediatePt = new OrientedPoint following egoSection.orientation from egoSpawnPt for globalParameters.OPT_distAhead
pedSpawnPt = new OrientedPoint right of intermediatePt by globalParameters.OPT_distSideways

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
	facing 90 deg relative to ego.heading,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

INIT_DIST = 50
TERM_DIST = 70

require (distance to intersection) > INIT_DIST
terminate when (distance to egoSpawnPt) > TERM_DIST