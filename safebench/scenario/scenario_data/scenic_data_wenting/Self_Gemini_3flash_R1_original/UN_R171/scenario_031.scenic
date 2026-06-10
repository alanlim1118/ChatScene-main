description = "Ego vehicle detects and autonomously brakes for a pedestrian crossing its path, avoiding a collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_AHEAD = Range(20, 30)
param OPT_DIST_SIDE = Range(5, 8)

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

pedSpawnPt = new OrientedPoint right of IntSpawnPt by globalParameters.OPT_DIST_SIDE

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

param EGO_SPEED = Range(8, 12)
param EGO_BRAKE_THRESHOLD = 15

behavior EgoBehavior(speed, trajectory, brake_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED, 
        egoTrajectory, 
        globalParameters.EGO_BRAKE_THRESHOLD
    )

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