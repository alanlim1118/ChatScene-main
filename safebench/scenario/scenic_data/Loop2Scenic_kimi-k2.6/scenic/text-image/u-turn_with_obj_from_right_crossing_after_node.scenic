description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

pedManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
pedInitLane = pedManeuver.startLane
pedSpawnPt = new OrientedPoint in pedInitLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior(egoTrajectory)

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian right of pedSpawnPt by 3,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

param TERM_DIST = 30

require (distance from egoSpawnPt to intersection) >= 20
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST or ego in intersection