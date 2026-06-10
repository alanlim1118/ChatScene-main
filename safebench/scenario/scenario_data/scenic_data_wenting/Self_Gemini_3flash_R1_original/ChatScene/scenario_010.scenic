description = "Ego vehicle turns left at an intersection; adversarial cyclist suddenly stops and dismounts, obstructing the path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

tempPt = new OrientedPoint ahead of egoSpawnPt by Range(5, 10)
bikeSpawnPt = new OrientedPoint left of tempPt by Range(2, 5)

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BRAKE_DIST = Range(5, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(2, 4)
param OPT_ADV_THRESHOLD = Range(15, 20)
param OPT_STOP_DIST = 0.5

behavior AdvBicycleBehavior():
    do CrossingBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_THRESHOLD) until (distance from self to egoManeuver.connectingLane) <= globalParameters.OPT_STOP_DIST
    take SetBrakeAction(1)
    take SetThrottleAction(0)
    while True:
        wait

bicycle = new Bicycle at bikeSpawnPt,
    facing 90 deg relative to bikeSpawnPt.heading,
    with behavior AdvBicycleBehavior()

monitor TrafficLightControl():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightControl()
require 10 <= (distance from egoSpawnPt to intersection) <= 30
require (distance from bikeSpawnPt to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > 70
terminate after 60 seconds