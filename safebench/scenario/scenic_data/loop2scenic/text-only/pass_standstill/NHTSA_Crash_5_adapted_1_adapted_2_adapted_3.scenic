description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 25)

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

param OPT_EGO_SPEED = Range(6, 9)
param OPT_SHARP_STEER = Range(-0.8, -0.6)

behavior EgoBehavior(speed, steer_val):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory) until (self in intersection)
	while self in intersection:
		take SetSteerAction(steer_val), SetThrottleAction(0.5)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_SHARP_STEER)

behavior StationaryBehavior():
    while True:
        wait

gnome = new Pedestrian at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBehavior()

param INIT_DIST = Range(15, 25)
param TERM_DIST = Range(30, 50)

require (distance from egoSpawnPt to advSpawnPt) >= 15
terminate when (distance from ego to advSpawnPt) > 50