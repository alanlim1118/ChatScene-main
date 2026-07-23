description = "Vehicle A turns left too sharply, clipping Vehicle B waiting at an intersection."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoDir = egoSpawnPt.heading

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advDir = advSpawnPt.heading

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

terminate when (distance from ego to egoSpawnPt) > 40
terminate after 15 seconds