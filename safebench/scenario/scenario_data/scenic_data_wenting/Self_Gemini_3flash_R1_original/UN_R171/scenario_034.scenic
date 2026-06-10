description = "Ego vehicle avoids a perpendicular cyclist emerging from an obstructed area at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param OPT_EGO_SPEED = Range(6, 10)
param OPT_BRAKE_DISTANCE = Range(10, 15)

behavior EgoBehavior(speed, brake_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

param OPT_ADV_MIN_SPEED = Range(1.5, 2.5)
param OPT_ADV_THRESHOLD = Range(15, 25)

behavior BicycleBehavior():
    do CrossingBehavior(ego, globalParameters.OPT_ADV_MIN_SPEED, globalParameters.OPT_ADV_THRESHOLD)

advBicycle = new Bicycle at advSpawnPt,
    with heading advSpawnPt.heading,
    with behavior BicycleBehavior(),
    with regionContainedIn None

obstruction = new Truck at advSpawnPt offset by 4 @ 8,
    with heading advSpawnPt.heading

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advBicycle, 100):
            setClosestTrafficLightStatus(advBicycle, "green")
        wait

require monitor TrafficLights()

terminate when (distance from ego to egoSpawnPt) > 50