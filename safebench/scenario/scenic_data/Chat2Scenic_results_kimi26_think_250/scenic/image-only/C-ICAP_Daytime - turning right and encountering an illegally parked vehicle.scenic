description = "Green VUT executes a right turn at an intersection where two illegally parked blue vehicles block the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
targetLane = egoManeuver.endLane
targetSection = targetLane.sections[0]
parked1SpawnPt = new OrientedPoint in targetSection.centerline
parked2SpawnPt = new OrientedPoint in targetSection.centerline

param OPT_EGO_SPEED = Range(5, 8)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdversaryBehavior():
    wait

adversary = new Car at parked1SpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

behavior StationaryBehavior():
    wait

adversary2 = new Car at parked2SpawnPt,
    with blueprint MODEL,
    with behavior StationaryBehavior()

EGO_INIT_MIN = 20
EGO_INIT_MAX = 35
TERM_DIST = 60

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightMonitor()
require EGO_INIT_MIN <= (distance to intersection) <= EGO_INIT_MAX
terminate when (distance to intersection) > TERM_DIST