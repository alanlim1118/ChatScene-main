description = "Ego vehicle passes another vehicle on a rural road and encroaches into oncoming traffic."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 30)
param OPT_ADV_DIST = Range(40, 80)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
egoLane = egoLaneSec.lane

oncomingLane = egoLaneSec.road.backwardLanes.lanes[0]
oncomingProjPt = oncomingLane.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following oncomingLane.orientation from oncomingProjPt for -globalParameters.OPT_ADV_DIST

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at AdvSpawnPt,
	with behavior AdversaryBehavior()

param OPT_LEADING_SPEED = Range(5, 8)

behavior LeadingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

leading = new Car at LeadingSpawnPt,
    with behavior LeadingBehavior()