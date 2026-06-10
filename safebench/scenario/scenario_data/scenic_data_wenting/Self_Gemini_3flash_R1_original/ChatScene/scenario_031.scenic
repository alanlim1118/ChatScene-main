description = "Ego performs right turn at intersection; crossing car speeds up, causing ego to brake abruptly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BRAKE_THRESHOLD = Range(6, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.8, 1.0)
param OPT_ADV_ACC_SPEED = globalParameters.OPT_ADV_SPEED * Uniform(1.8, 2.0)
param OPT_ADV_DISTANCE = Range(15, 25)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory) until (distance from self to ego < globalParameters.OPT_ADV_DISTANCE)
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_ACC_SPEED, trajectory=advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_ACC_SPEED)

advAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

EGO_INIT_DIST = [10, 25]
ADV_INIT_DIST = [10, 25]
TERM_DIST = 70

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advAgent to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST