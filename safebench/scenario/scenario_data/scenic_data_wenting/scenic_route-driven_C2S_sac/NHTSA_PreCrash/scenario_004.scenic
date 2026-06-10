description = "Ego vehicle runs a stop sign at an intersection in a rural area with a low speed limit."
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

TERM_DIST = 70

monitor TrafficLightManager():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightManager()
require 20 <= (distance to intersection) <= 30
terminate when (distance from egoSpawnPt to ego) > TERM_DIST
