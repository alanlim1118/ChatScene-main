description = "Vehicle runs a red light after seeing it turn yellow."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

monitor TrafficLightControl():
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, "green")
    wait until withinDistanceToTrafficLight(ego, 25)
    setClosestTrafficLightStatus(ego, "yellow")
    wait for 2 seconds
    setClosestTrafficLightStatus(ego, "red")
    while True:
        wait

require monitor TrafficLightControl()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
terminate when (ego in egoManeuver.endLane) and (distance from ego to egoSpawnPt > 20)
