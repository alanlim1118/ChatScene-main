description = "Ego vehicle turns left at an intersection across the path of an oncoming vehicle, requiring collision avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = Range(12, 15)

behavior EgoBehavior(trajectory, speed, safety_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with behavior AdversaryBehavior(advTrajectory, globalParameters.ADV_SPEED)


monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, 'green')
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, 'green')
        wait

require monitor TrafficLightMonitor()
terminate when (distance from ego to egoSpawnPt) > 80