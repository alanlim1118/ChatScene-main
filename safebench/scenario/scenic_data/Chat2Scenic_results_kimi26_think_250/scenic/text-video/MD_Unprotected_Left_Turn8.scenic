description = "Ego vehicle performs a left turn at an intersection yielding to oncoming and crossing traffic and a pedestrian."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingInitLane.centerline

tempManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.reverseManeuvers))
rightRoad = tempManeuver.endLane.road
rightLane = Uniform(*rightRoad.lanes)
rightSpawnPt1 = new OrientedPoint in rightLane.centerline
rightSpawnPt2 = new OrientedPoint in rightLane.centerline
rightSpawnPt3 = new OrientedPoint in rightLane.centerline
pedSpawnPt = rightLane.centerline[-1]

param EGO_SPEED = Range(7, 10)
param EGO_YIELD_DIST = Range(8, 12)

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyCars(self, globalParameters.EGO_YIELD_DIST) or withinDistanceToAnyPedestrians(self, globalParameters.EGO_YIELD_DIST):
		take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at oncomingSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(oncomingTrajectory)

behavior CrossingCarBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

crossingAdversary = new Car at rightSpawnPt1,
	with blueprint MODEL,
	with behavior CrossingCarBehavior()

behavior CrossingAdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

crossingAdv = new Car at rightSpawnPt2,
	with blueprint MODEL,
	with behavior CrossingAdvBehavior()

behavior CrossIntersectionBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

crossingCar = new Car at rightSpawnPt3,
	with blueprint MODEL,
	with behavior CrossIntersectionBehavior()

param PEDESTRIAN_MIN_SPEED = 1.0
param PEDESTRIAN_THRESHOLD = 20

behavior PedestrianCrossingBehavior():
    do CrossingBehavior(ego, PEDESTRIAN_MIN_SPEED, PEDESTRIAN_THRESHOLD)

pedestrian = new Pedestrian at pedSpawnPt,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

EGO_INIT_DIST = [20, 35]
ADV_INIT_DIST = [10, 40]
TERM_DIST = 80

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from crossingAdversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from crossingAdv to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from crossingCar to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST