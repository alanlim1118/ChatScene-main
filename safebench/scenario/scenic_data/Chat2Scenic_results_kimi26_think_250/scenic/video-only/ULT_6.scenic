description = "Ego vehicle executes a left turn at a four-way intersection while yielding to cross-traffic from opposing and left arms."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

oppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
oppInitLane = oppManeuver.startLane
oppTrajectory = [oppInitLane, oppManeuver.connectingLane, oppManeuver.endLane]
oppSpawnPt = new OrientedPoint in oppInitLane.centerline

leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers)).conflictingManeuvers))
leftInitLane = leftManeuver.startLane
leftTrajectory = [leftInitLane, leftManeuver.connectingLane, leftManeuver.endLane]
leftSpawnPt = new OrientedPoint in leftInitLane.centerline

param OPT_EGO_SPEED = Range(3, 5)
param OPT_EGO_ACCELERATED_SPEED = globalParameters.OPT_EGO_SPEED + Uniform(2, 3)
param OPT_EGO_YIELD_DIST = Range(8, 10)
OPT_EGO_DECISION_DEGREE = 35 deg

behavior EgoBehavior():
    initialDir = egoSpawnPt.heading
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_EGO_YIELD_DIST):
        currentDir = self.heading
        if (abs(currentDir - initialDir) < OPT_EGO_DECISION_DEGREE):
            take SetThrottleAction(0)
            take SetBrakeAction(1)
        else:
            do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_ACCELERATED_SPEED)
            abort
    terminate

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

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