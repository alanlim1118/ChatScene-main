description = "Ego vehicle traverses an intersection while a pedestrian crosses to impact its path, requiring 4-second pre-collision detection and intervention."
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
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

intersectionPt = new OrientedPoint in egoManeuver.connectingLane.centerline
pedSpawnPt = new OrientedPoint right of intersectionPt by 5, facing intersectionPt

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

monitor TrafficLights():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 100):
			setClosestTrafficLightStatus(ego, 'green')
		wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to intersectionPt) <= 40
terminate when (ego intersects ped)
terminate when (distance from ego to egoSpawnPt) > 50
