description = "Pedestrian struck by vehicle while crossing multi-lane road; driver failed to see pedestrian."
param map = localPath('../../maps/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param distAhead = Range(20, 30)
param distSideways = Range(5, 10)

multiLaneSections = []
for lane in network.lanes:
	for sec in lane.sections:
		if sec._laneToRight is not None or sec._laneToLeft is not None:
			multiLaneSections.append(sec)

egoSection = Uniform(*multiLaneSections)
egoSpawnPt = new OrientedPoint in egoSection.centerline

intermediatePt = new OrientedPoint following egoSection.orientation from egoSpawnPt for globalParameters.distAhead
pedSpawnPt = new OrientedPoint right of intermediatePt by globalParameters.distSideways

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(speed):
	do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED)

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