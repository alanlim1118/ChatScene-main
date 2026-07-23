description = "Ego vehicle stops behind a leading left-turning vehicle at a four-way intersection while yielding to opposing traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint on advInitLane.centerline

egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)

oppAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advManeuver.conflictingManeuvers))
oppAdvInitLane = oppAdvManeuver.startLane
oppAdvSpawnPt = new OrientedPoint on oppAdvInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
oppAdvTrajectory = [oppAdvInitLane, oppAdvManeuver.connectingLane, oppAdvManeuver.endLane]

param OPT_EGO_SPEED = Range(6, 10)
param OPT_EGO_STOP_DISTANCE = Range(3, 5)

behavior EgoBehavior(speed, stop_distance):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, stop_distance)
    while True:
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_STOP_DISTANCE)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

behavior OpponentBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

opponent = new Car at oppAdvSpawnPt,
	with behavior OpponentBehavior(oppAdvTrajectory)

require 5 <= (distance from ego to adversary) <= 10
require (distance from opponent to intersection) <= 30
require (distance from adversary to intersection) <= 30
terminate when (distance from opponent to intersection) > 40 and (distance from adversary to intersection) > 40