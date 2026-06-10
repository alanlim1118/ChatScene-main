description = "Vehicle turning at an intersection loses control on a wet road and runs off the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type in {ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN}, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

monitor TrafficLightManager():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightManager()
require 20 <= (distance from egoSpawnPt to intersection) <= 40
terminate when (ego not in network.roads) and (distance from ego to egoSpawnPt > 10)
