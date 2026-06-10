description = "Ego avoids collision with red-light runner at intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint on advInitLane.centerline

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE_THRESHOLD = 12

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.EGO_BRAKE_THRESHOLD):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(10, 15)
param OPT_ADV_TRIGGER_DIST = Range(35, 50)
param OPT_ADV_DRIVE_TIME = Range(1.5, 2.5)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_TRIGGER_DIST
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory) for globalParameters.OPT_ADV_DRIVE_TIME seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1.0)
    do WaitBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            setClosestTrafficLightStatus(AdvAgent, "red")
        wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 30 <= (distance from advSpawnPt to intersection) <= 50

terminate when (distance from ego to egoSpawnPt) > 100