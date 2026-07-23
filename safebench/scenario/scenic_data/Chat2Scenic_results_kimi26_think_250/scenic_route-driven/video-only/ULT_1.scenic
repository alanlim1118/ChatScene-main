description = "Ego vehicle stops behind a leading left-turning vehicle at a four-way intersection while yielding to opposing traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = advManeuver.intersection
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(5, 10)

oppAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advManeuver.conflictingManeuvers))
oppAdvInitLane = oppAdvManeuver.startLane
oppAdvSpawnPt = new OrientedPoint on oppAdvInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
oppAdvTrajectory = [oppAdvInitLane, oppAdvManeuver.connectingLane, oppAdvManeuver.endLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

behavior OpponentBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

opponent = new Car at oppAdvSpawnPt,
	with behavior OpponentBehavior(oppAdvTrajectory)

require 5 <= (distance from ego to adversary) <= 10
require (distance from opponent to intersection) <= 30
require (distance from adversary to intersection) <= 30
terminate when (distance from opponent to intersection) > 40 and (distance from adversary to intersection) > 40