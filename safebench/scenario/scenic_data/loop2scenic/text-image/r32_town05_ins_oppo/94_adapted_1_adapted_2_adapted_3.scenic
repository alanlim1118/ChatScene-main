description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearNoon"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

adv1Maneuver = egoManeuver
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(15, 25)

adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
require adv2Maneuver.startLane.road != egoInitLane.road
require adv2Maneuver.startLane.road in intersection.roads
adv2InitLane = adv2Maneuver.startLane
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv2Trajectory = [adv2Maneuver.startLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]

param EGO_SPEED = Range(7, 10)

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
	with blueprint MODEL,
	with behavior Adv2Behavior(adv2Trajectory)

param ADV1_SPEED = Range(7, 10)

behavior Adv1Behavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV1_SPEED, trajectory=trajectory)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior Adv1Behavior(adv1Trajectory)

buildingSpawnPt = new OrientedPoint in intersection
building = new Prop at buildingSpawnPt,
    with blueprint 'static.prop.kiosk_01'

param EGO_ADV1_DIST = Range(15, 25)

require (distance from egoSpawnPt to adv1SpawnPt) >= 15
require (distance from egoSpawnPt to adv1SpawnPt) <= 30

terminate when (ego in intersection) and (distance from ego to intersection > 10)