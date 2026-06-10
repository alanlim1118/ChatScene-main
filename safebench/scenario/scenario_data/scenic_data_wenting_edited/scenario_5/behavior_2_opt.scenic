description = "Ego vehicle avoids collision with an adversarial agent running a red light and turning right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

require advSpawnPt is left of egoSpawnPt
require advSpawnPt is ahead of egoSpawnPt

param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_THRESHOLD = Range(10, 15)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_BRAKE_THRESHOLD):
        take SetBrakeAction(1)
        take SetThrottleAction(0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(10, 15)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "red")
        wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20
terminate when (distance from ego to egoSpawnPt) > 50