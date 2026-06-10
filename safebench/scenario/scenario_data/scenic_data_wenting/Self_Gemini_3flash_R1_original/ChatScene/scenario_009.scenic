description = "Ego vehicle turns right at intersection; adversarial pedestrian suddenly crosses and stops, blocking path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: (i.is4Way or i.is3Way) and any(m.type is ManeuverType.RIGHT_TURN for m in i.maneuvers), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

tempFrontPt = new OrientedPoint following egoInitLane.orientation from egoInitLane.centerline[-1] for 2
pedSpawnPt = new OrientedPoint left of tempFrontPt by 5

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(1.0, 2.0)
param OPT_ADV_DISTANCE = Range(15, 20)
OPT_STOP_DISTANCE = 1.0

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
    do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
    take SetWalkingSpeedAction(0)

AdvAgent = new Pedestrian at pedSpawnPt,
    facing toward egoManeuver.connectingLane.centerline[0.5],
    with regionContainedIn None,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, egoManeuver.connectingLane.centerline[0.5], OPT_STOP_DISTANCE)

EGO_INIT_DIST = [20, 25]
TERM_DIST = 50

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST