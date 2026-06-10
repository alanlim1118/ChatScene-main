description = "Ego drives straight through an intersection when a crossing vehicle runs a red light and accelerates, forcing ego to perform a collision avoidance maneuver."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(7, 10)
param BRAKE_DIST = Range(5, 10)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.EGO_SPEED * Range(0.7, 0.9)
param OPT_ADV_ACC_SPEED = globalParameters.EGO_SPEED * Range(1.5, 2.0)
param OPT_ADV_DISTANCE = Range(15, 25)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to ego < globalParameters.OPT_ADV_DISTANCE)
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_ACC_SPEED, advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_ACC_SPEED)

advVehicle = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior(),
    with regionContainedIn None

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advVehicle, 100):
            setClosestTrafficLightStatus(advVehicle, "red")
        wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20
terminate when (distance from ego to advVehicle) > 30 and (distance from ego to egoSpawnPt) > 50