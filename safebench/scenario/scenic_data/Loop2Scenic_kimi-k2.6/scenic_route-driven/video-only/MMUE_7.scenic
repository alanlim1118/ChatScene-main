description = "Using map ../../maps/Town10HD.xodr with carla map Town10HD and weather ClearNoon"
param map = localPath('../../maps/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuverCands = [m for m in egoInitLane.maneuvers if m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way]
intersection = egoManeuverCands[0].intersection
egoManeuver = Uniform(*[m for m in egoManeuverCands if m.intersection is intersection])

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
require advManeuver.startLane.road != egoInitLane.road
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param OPT_EGO_INIT_DIST = Range(30, 40)
param OPT_ADV_INIT_DIST = Range(10, 20)

require (distance from egoSpawnPt to intersection) >= 30
require (distance from advSpawnPt to intersection) >= 10

terminate when (ego in egoManeuver.endLane and adversary in advManeuver.endLane) or (distance from ego to intersection) > 100