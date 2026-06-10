description = "Rear-end collision: A vehicle fails to stop for a stopped vehicle at a red light."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE = Range(10, 15)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

intersections = filter(lambda i: i.isSignalized and egoInitLane in i.incomingLanes, network.intersections)
targetIntersection = Uniform(*intersections)

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

behavior AdversaryBehavior():
    while True:
        wait

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(adversary, 30):
            setClosestTrafficLightStatus(adversary, 'red')
        wait

require monitor TrafficLightMonitor()
require (distance from advSpawnPt to targetIntersection) <= 20
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 15

terminate when (distance from ego to egoSpawnPt) > 50