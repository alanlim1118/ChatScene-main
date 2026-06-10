description = "Ego vehicle turning left at a signaled urban intersection encounters a pedestrian in the crosswalk."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

targetLane = egoManeuver.endLane
pedSpawnPt = new OrientedPoint at targetLane.centerline[0]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
	facing 90 deg relative to pedSpawnPt.heading,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightMonitor()
require 15 <= (distance from egoSpawnPt to intersection) <= 25
terminate when (distance from ego to egoSpawnPt) > 40
