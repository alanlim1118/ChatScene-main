description = "Ego vehicle turns right at an urban intersection, encountering a pedalcyclist, under a 25 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(m.type is ManeuverType.RIGHT_TURN for m in i.maneuvers), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

bicycleManeuver = Uniform(*egoManeuver.conflictingManeuvers)
bicycleInitLane = bicycleManeuver.startLane
bicycleSpawnPt = new OrientedPoint in bicycleInitLane.centerline
bicycleTrajectory = [bicycleInitLane, bicycleManeuver.connectingLane, bicycleManeuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param BICYCLE_SPEED = Range(4, 7)

behavior BicycleBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.BICYCLE_SPEED, trajectory=trajectory)

bicycle = new Bicycle at bicycleSpawnPt,
	with behavior BicycleBehavior(bicycleTrajectory)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(bicycle, 100):
            setClosestTrafficLightStatus(bicycle, "green")
        wait

require monitor TrafficLights()
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from bicycleSpawnPt to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > 50