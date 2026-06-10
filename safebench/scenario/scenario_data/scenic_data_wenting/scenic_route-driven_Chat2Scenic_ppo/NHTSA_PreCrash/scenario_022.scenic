description = "Ego vehicle approaches a stopped lead vehicle at an urban intersection with a 35 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

param OPT_DISTANCE = Range(5, 10)

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

adversary = new Car at advSpawnPt,
	with blueprint MODEL

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 10
terminate when (distance from ego to adversary) < 2.5
