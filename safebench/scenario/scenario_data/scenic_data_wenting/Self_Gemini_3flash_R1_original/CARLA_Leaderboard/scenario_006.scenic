description = "Ego vehicle turns at an intersection, yielding to crossing bicycles."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

crossingManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers)

bike1Maneuver = Uniform(*crossingManeuvers)
bike1InitLane = bike1Maneuver.startLane
bike1SpawnPt = new OrientedPoint in bike1InitLane.centerline
bike1Trajectory = [bike1InitLane, bike1Maneuver.connectingLane, bike1Maneuver.endLane]

bike2Maneuver = Uniform(*crossingManeuvers)
bike2InitLane = bike2Maneuver.startLane
bike2SpawnPt = new OrientedPoint in bike2InitLane.centerline
bike2Trajectory = [bike2InitLane, bike2Maneuver.connectingLane, bike2Maneuver.endLane]

param EGO_SPEED = Range(6, 8)
param SAFETY_DIST = Range(10, 15)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param BIKE_SPEED = Range(4, 6)

behavior BicycleBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.BIKE_SPEED, trajectory=trajectory)

bike = new Bicycle at bike1SpawnPt,
    with behavior BicycleBehavior(bike1Trajectory)

param ADV_SPEED = Range(4, 6)

behavior BicycleBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

bike1 = new Bicycle at bike1SpawnPt,
    with behavior BicycleBehavior(bike1Trajectory),
    with regionContainedIn None

bike2 = new Bicycle at bike2SpawnPt,
    with behavior BicycleBehavior(bike2Trajectory),
    with regionContainedIn None

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(bike1, 100):
            setClosestTrafficLightStatus(bike1, "green")
        if withinDistanceToTrafficLight(bike2, 100):
            setClosestTrafficLightStatus(bike2, "green")
        wait

require monitor TrafficLights()
require 20 <= (distance from egoSpawnPt to intersection) <= 30
require 10 <= (distance from bike1SpawnPt to intersection) <= 20
require 10 <= (distance from bike2SpawnPt to intersection) <= 20

terminate when (distance from ego to egoSpawnPt) > 50