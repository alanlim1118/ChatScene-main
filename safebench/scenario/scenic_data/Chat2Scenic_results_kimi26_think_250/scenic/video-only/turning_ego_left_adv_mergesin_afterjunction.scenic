description = "Ego vehicle turning left at an intersection is struck by a taxi cutting across multiple lanes."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
sedanSpawnPt = new OrientedPoint in egoInitLane.centerline
egoSpawnPt = new OrientedPoint behind sedanSpawnPt by Range(5, 10)
advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(10, 15)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with behavior AdvBehavior()

param EGO_INT_DIST = [15, 30]
param ADV_PATH_DIST = [5, 20]

monitor ReproducibilityMonitor():
    freezeTrafficLights()
    while True:
        wait

require monitor ReproducibilityMonitor()
require EGO_INT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INT_DIST[1]
require ADV_PATH_DIST[0] <= (distance from advSpawnPt to sedanSpawnPt) <= ADV_PATH_DIST[1]
terminate when ego intersects adversary