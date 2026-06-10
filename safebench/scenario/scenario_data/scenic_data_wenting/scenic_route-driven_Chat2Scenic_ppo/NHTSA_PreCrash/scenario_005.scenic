description = "Vehicle turns at a rural intersection at night under clear conditions with a 25 mph speed limit, then departs the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require (distance to intersection) > 15
terminate when (ego not in network.roads) and (distance from egoSpawnPt > 40)
