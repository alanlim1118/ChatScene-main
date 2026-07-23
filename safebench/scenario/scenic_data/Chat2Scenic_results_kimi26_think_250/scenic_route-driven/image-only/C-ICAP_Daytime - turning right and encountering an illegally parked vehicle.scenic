description = "Green VUT executes a right turn at an intersection where two illegally parked blue vehicles block the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
targetLane = egoManeuver.endLane
targetSection = targetLane.sections[0]
parked1SpawnPt = new OrientedPoint in targetSection.centerline
parked2SpawnPt = new OrientedPoint in targetSection.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

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