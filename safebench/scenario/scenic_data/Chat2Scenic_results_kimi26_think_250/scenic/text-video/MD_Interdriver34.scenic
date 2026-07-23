description = "Ego vehicle turns left at a four-way intersection while two oncoming vehicles turn right onto the same western road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advManeuver.startLane.centerline
adv2SpawnPt = new OrientedPoint following roadDirection from advSpawnPt for -Range(10, 20)

param OPT_EGO_SPEED = Range(3, 5)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(trajectory=advTrajectory)

adversary2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(trajectory=advTrajectory)

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]
ADV2_INIT_DIST = [25, 40]

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV2_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV2_INIT_DIST[1]
terminate when (ego in egoManeuver.endLane) and (distance from ego to intersection) > 10