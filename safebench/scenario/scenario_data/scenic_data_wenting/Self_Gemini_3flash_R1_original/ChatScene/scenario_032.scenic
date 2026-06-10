description = "Ego vehicle turning right encounters adversarial car blocking lane by sudden braking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advInitLane = egoManeuver.endLane
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.start for Range(5, 10)
advTrajectory = [advInitLane]

param EGO_SPEED = Range(7, 10)
param BRAKE_DIST = Range(5, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE_THRESHOLD = Range(10, 15)

behavior AdversarialBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory) until (distance from self to ego) < globalParameters.ADV_BRAKE_THRESHOLD
    while True:
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)

adversarial = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversarialBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversarial, 100):
            setClosestTrafficLightStatus(adversarial, "green")
        wait

require monitor TrafficLights()
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 5 <= (distance from advSpawnPt to intersection) <= 20

terminate when (distance from ego to egoSpawnPt) > 100
terminate after 40 seconds