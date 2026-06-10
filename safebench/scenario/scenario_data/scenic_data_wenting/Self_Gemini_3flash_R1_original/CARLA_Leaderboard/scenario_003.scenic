description = "Ego vehicle performs a right turn at an intersection, yielding to crossing traffic, at signalized and non-signalized junctions."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.RIGHT_TURN for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(6, 8)
param YIELD_THRESHOLD = 15

behavior EgoBehavior(trajectory, target_speed, yield_threshold):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        egoTrajectory,
        globalParameters.EGO_SPEED,
        globalParameters.YIELD_THRESHOLD
    )

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

EGO_INIT_DIST = [20, 30]
ADV_INIT_DIST = [20, 30]
TERM_DIST = 80

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 50):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLights()
require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]
terminate when (distance from ego to egoSpawnPt) > TERM_DIST