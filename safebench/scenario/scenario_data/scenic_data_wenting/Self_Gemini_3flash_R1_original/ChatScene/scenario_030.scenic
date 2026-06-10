description = "Ego performs unprotected left turn, yielding to erratic oncoming vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param OPT_EGO_SPEED = Range(3, 5)
param OPT_EGO_YIELD_DIST = Range(15, 20)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.2, 1.5)

behavior ErraticBehavior(speed, traj):
    while True:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=traj) for Range(0.8, 1.5) seconds
        take SetSteerAction(Range(-0.5, 0.5))
        wait for Range(0.1, 0.4) seconds

AdvAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior ErraticBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

require distance to intersection < 15
require distance from AdvAgent to intersection < 15

monitor TrafficLightMonitor():
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, 'green')
    setClosestTrafficLightStatus(AdvAgent, 'green')
    wait until False

require monitor TrafficLightMonitor()

terminate when (distance from ego to egoSpawnPt) > 10 and (ego in egoManeuver.endLane or distance from ego to AdvAgent > 50)