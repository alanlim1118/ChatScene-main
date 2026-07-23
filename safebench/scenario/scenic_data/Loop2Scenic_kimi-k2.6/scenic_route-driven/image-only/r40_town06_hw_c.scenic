description = "Ego vehicle approaches a stopped lead vehicle at an urban intersection with a 35 mph speed limit."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(5, 10)

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