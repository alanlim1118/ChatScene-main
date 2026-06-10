description = "Vehicle runs a red light after seeing it turn yellow."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.isSignalized, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

monitor TrafficLightControl():
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, "green")
    wait until withinDistanceToTrafficLight(ego, 25)
    setClosestTrafficLightStatus(ego, "yellow")
    wait for 2 seconds
    setClosestTrafficLightStatus(ego, "red")
    while True:
        wait

require monitor TrafficLightControl()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
terminate when (ego in egoManeuver.endLane) and (distance from ego to egoSpawnPt > 20)