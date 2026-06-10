description = "Ego performs an unprotected left turn, evading an oncoming car that unexpectedly accelerates due to throttle malfunction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)

egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(5, 8)
param EVASION_DIST = 15

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.EVASION_DIST):
        take SetBrakeAction(1.0)
        take SetSteerAction(-0.3)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)
param MALFUNCTION_THROTTLE = Range(0.8, 1.0)
param TRIGGER_DISTANCE = Range(10, 20)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory) until (distance from self to intersection) <= globalParameters.TRIGGER_DISTANCE
    do ConstantThrottleBehavior(globalParameters.MALFUNCTION_THROTTLE)

advCar = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

param SUCCESS_DIST = 80
INIT_DIST_TO_ADV = [20, 40]

monitor TrafficLightControl:
    freezeTrafficLights()
    while True:
        setClosestTrafficLightStatus(ego, 'green')
        setClosestTrafficLightStatus(advCar, 'green')
        wait

require monitor TrafficLightControl()
require INIT_DIST_TO_ADV[0] <= (distance from ego to advCar) <= INIT_DIST_TO_ADV[1]

terminate when ego intersects advCar
terminate when (distance to egoSpawnPt) > globalParameters.SUCCESS_DIST