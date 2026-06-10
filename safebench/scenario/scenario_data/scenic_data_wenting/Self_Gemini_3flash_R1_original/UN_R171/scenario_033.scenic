description = "Ego vehicle traverses an intersection while a pedestrian crosses to impact its path, requiring 4-second pre-collision detection and intervention."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

intersectionPt = new OrientedPoint in egoManeuver.connectingLane.centerline
pedSpawnPt = new OrientedPoint right of intersectionPt by 5, facing intersectionPt

param EGO_SPEED = Range(6, 9)
param EGO_BRAKE = 1.0
EGO_SAFE_DIST = 30

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

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