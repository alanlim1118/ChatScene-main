description = "Ego vehicle avoids a perpendicular cyclist emerging from an obstructed area at an intersection."
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
intersection = egoManeuver.intersection
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

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
