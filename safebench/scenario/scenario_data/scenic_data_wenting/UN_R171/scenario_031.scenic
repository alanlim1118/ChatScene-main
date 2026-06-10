description = "Ego vehicle detects and autonomously brakes for a pedestrian crossing its path, avoiding a collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_AHEAD = Range(20, 30)
param OPT_DIST_SIDE = Range(5, 8)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

pedSpawnPt = new OrientedPoint right of IntSpawnPt by globalParameters.OPT_DIST_SIDE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to IntSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 20 <= (distance from egoSpawnPt to intersection) <= 30
terminate when (distance to egoSpawnPt) > 50
