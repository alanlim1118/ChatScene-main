description = "Vehicle A turns left too sharply, clipping Vehicle B waiting at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

intersections = filter(lambda i: i.is4Way and egoInitLane in i.incomingLanes, network.intersections)
intersection = Uniform(*intersections)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "red")
        wait

require monitor TrafficLights()
require 15 <= (distance from egoSpawnPt to intersection) <= 25
require (distance from advSpawnPt to intersection) <= 5

TERM_DIST = 40
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
terminate after 15 seconds
