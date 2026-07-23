description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather CloudyNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

adv1Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

adv2SpawnPt = new OrientedPoint behind egoSpawnPt by Range(5, 15)

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior(egoTrajectory)

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=trajectory)

adv2 = new Car at adv2SpawnPt,
    facing roadDirection,
    with regionContainedIn None,
    with behavior Adv2Behavior(egoTrajectory)

param ADV1_SPEED = Range(6, 8)

behavior Adv1Behavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV1_SPEED, trajectory=trajectory)

adv1 = new Car at adv1SpawnPt,
    facing roadDirection,
    with regionContainedIn None,
    with behavior Adv1Behavior(adv1Trajectory)

param INIT_DIST = Range(40, 60)
param TERM_DIST = Range(30, 50)

require (distance from egoSpawnPt to intersection) >= 40
require (distance from adv1SpawnPt to intersection) >= 40

terminate when (distance from ego to egoSpawnPt) > (distance from egoSpawnPt to intersection) + 30