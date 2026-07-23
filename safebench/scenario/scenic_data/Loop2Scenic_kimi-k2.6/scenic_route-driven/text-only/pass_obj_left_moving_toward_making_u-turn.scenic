description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
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

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
require advManeuver.startLane != egoInitLane
require advManeuver.startLane.road != egoInitLane.road

advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

require 10 <= (distance from egoSpawnPt to intersection) <= 20
require 30 <= (distance from advSpawnPt to intersection) <= 40
terminate when (distance from ego to adversary) > 50 and (distance from ego to intersection) > 30