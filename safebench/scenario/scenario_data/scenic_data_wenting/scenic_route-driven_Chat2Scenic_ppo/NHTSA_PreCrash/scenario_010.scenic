description = "Ego vehicle turns right at an urban intersection, encountering a pedalcyclist, under a 25 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

bicycleManeuver = Uniform(*egoManeuver.conflictingManeuvers)
bicycleInitLane = bicycleManeuver.startLane
bicycleSpawnPt = new OrientedPoint in bicycleInitLane.centerline
bicycleTrajectory = [bicycleInitLane, bicycleManeuver.connectingLane, bicycleManeuver.endLane]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_BICYCLE_SPEED = Range(4, 7)

behavior BicycleBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_BICYCLE_SPEED, trajectory=trajectory)

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
