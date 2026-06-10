description = "Ego vehicle detects a stationary pedestrian near its lane and performs autonomous braking to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

param OPT_DIST_AHEAD = Range(20, 35)
targetPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

pedSpawnPt = new OrientedPoint right of targetPt by Range(2.1, 2.6)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

behavior StationaryBehavior():
    take SetWalkingSpeedAction(0)
    while True:
        wait

ped = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with behavior StationaryBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 20 <= (distance from egoSpawnPt to pedSpawnPt) <= 40
terminate when (distance from ego to pedSpawnPt) > 50
terminate after 30 seconds
