description = "Ego vehicle follows a blue lead vehicle making a left turn at a four-way intersection as a yellow adversary approaches straight from the opposing direction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advManeuver.startLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(3, 6)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=egoTrajectory)

adversary = new Car ahead of egoSpawnPt by Range(10, 30),
    with blueprint MODEL,
    with behavior AdvBehavior()

param OPT_ADV_STRAIGHT_SPEED = Range(7, 10)

behavior StraightAdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_STRAIGHT_SPEED, trajectory=trajectory)

adversary2 = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior StraightAdvBehavior(advTrajectory)

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]
TERM_DIST = 70

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST