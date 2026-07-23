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
egoManeuver = Uniform(*egoManeuverCands)
intersection = egoManeuver.intersection

advManeuverCands = [am for m in egoManeuverCands for am in m.intersection.maneuvers if am.type is ManeuverType.RIGHT_TURN]
advManeuver = Uniform(*advManeuverCands)
require advManeuver.startLane.road != egoInitLane.road
require advManeuver.endLane.road == egoInitLane.road

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

param OPT_ADV_SPEED = Range(7, 10)

require 10 <= (distance from advSpawnPt to intersection) <= 20
require 0 <= (distance from egoSpawnPt to intersection) <= 5

terminate when (distance from adversary to advManeuver.endLane.centerline) < 2 and (distance from adversary to intersection) > 15