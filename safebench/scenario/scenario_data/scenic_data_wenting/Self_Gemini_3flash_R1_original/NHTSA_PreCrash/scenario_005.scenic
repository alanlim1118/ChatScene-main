description = "Vehicle turns at a rural intersection at night under clear conditions with a 25 mph speed limit, then departs the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

intersection = Uniform(*network.intersections)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

param EGO_SPEED = Range(7, 10)
param DEPART_STEER = 0.3
param DEPART_THROTTLE = 0.5

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    while True:
        take SetSteerAction(globalParameters.DEPART_STEER), SetThrottleAction(globalParameters.DEPART_THROTTLE)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require (distance to intersection) > 15
terminate when (ego not in network.roads) and (distance from egoSpawnPt > 40)