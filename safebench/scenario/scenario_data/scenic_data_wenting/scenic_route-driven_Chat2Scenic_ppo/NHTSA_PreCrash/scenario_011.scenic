description = "Ego vehicle drives straight in an urban area and encounters a pedalcyclist at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.conflictingManeuvers, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

bikeManeuver = Uniform(*egoManeuver.conflictingManeuvers)
bikeInitLane = bikeManeuver.startLane
bikeTrajectory = [bikeInitLane, bikeManeuver.connectingLane, bikeManeuver.endLane]
bikeSpawnPt = new OrientedPoint in bikeInitLane.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_BIKE_SPEED = Range(4, 7)

behavior BikeBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_BIKE_SPEED, trajectory=trajectory)

bike = new Bicycle at bikeSpawnPt,
    with behavior BikeBehavior(bikeTrajectory)

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, 'green')
        if withinDistanceToTrafficLight(bike, 100):
            setClosestTrafficLightStatus(bike, 'green')
        wait

require monitor TrafficLightMonitor()
require 20 <= (distance from egoSpawnPt to intersection) <= 30
require 15 <= (distance from bikeSpawnPt to intersection) <= 25

TERM_DIST = 60
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
