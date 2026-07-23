description = "Passing vehicle crosses dashed center line into opposing lane creating head-on conflict with oncoming vehicle."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 20)
param OPT_ONCOMING_DIST = Range(30, 60)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

advLaneSec = egoLaneSec._laneToLeft
oppProjPt = advLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from oppProjPt for globalParameters.OPT_ONCOMING_DIST

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior(target_speed):
	do FollowLaneBehavior(target_speed=target_speed)

adversary = new Car at AdvSpawnPt,
	with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED)

param OPT_OPPOSITE_CAR_SPEED = Range(8, 12)

behavior OppositeCarBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

oppositeCar = new Car behind AdvSpawnPt by Range(20, 40),
    facing roadDirection,
    with behavior OppositeCarBehavior(globalParameters.OPT_OPPOSITE_CAR_SPEED)

require (distance from egoSpawnPt to intersection) > 0
terminate when (distance from ego to adversary) > 80