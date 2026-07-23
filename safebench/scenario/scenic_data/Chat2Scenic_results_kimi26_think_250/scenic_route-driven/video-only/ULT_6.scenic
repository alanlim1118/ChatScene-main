description = "Ego vehicle executes a left turn at a four-way intersection while yielding to cross-traffic from opposing and left arms."
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

oppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
oppInitLane = oppManeuver.startLane
oppTrajectory = [oppInitLane, oppManeuver.connectingLane, oppManeuver.endLane]
oppSpawnPt = new OrientedPoint in oppInitLane.centerline

leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers)).conflictingManeuvers))
leftInitLane = leftManeuver.startLane
leftTrajectory = [leftInitLane, leftManeuver.connectingLane, leftManeuver.endLane]
leftSpawnPt = new OrientedPoint in leftInitLane.centerline

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL

param OPT_OPP_SPEED = Range(8, 12)

behavior OppBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_OPP_SPEED, trajectory=oppTrajectory)
    terminate

oppCar = new Car at oppSpawnPt,
    facing oppSpawnPt.heading,
    with regionContainedIn None,
    with behavior OppBehavior()

param OPT_LEFT_SPEED = Range(8, 12)

behavior LeftBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_LEFT_SPEED, trajectory=leftTrajectory)
    terminate

leftCar = new Car at leftSpawnPt,
    facing leftSpawnPt.heading,
    with regionContainedIn None,
    with behavior LeftBehavior()

monitor LeftCarMonitor():
    wait until leftCar intersects intersection
    wait until leftCar in leftManeuver.endLane

require (distance from egoSpawnPt to intersection) >= 15
require monitor LeftCarMonitor()
terminate when ego in egoManeuver.endLane