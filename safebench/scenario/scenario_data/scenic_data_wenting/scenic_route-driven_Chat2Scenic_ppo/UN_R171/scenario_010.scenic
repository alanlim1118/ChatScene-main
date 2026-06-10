description = "Ego vehicle encounters a stationary oncoming vehicle in its lane after rounding a curve in a non-highway environment."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(30, 50)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST,
             facing (180 deg) relative to roadDirection

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

behavior AdversaryBehavior():
    while True:
        wait

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param TERMINATE_DISTANCE = 80
param MAX_TIME = 40

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to advSpawnPt) <= 50

terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATE_DISTANCE
terminate after globalParameters.MAX_TIME seconds
