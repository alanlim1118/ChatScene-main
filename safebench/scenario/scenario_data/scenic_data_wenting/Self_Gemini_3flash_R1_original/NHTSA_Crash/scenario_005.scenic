description = "Vehicle A turns left too sharply, clipping Vehicle B waiting at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way and any(m.type is ManeuverType.LEFT_TURN for m in i.maneuvers), network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoDir = egoSpawnPt.heading

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advDir = advSpawnPt.heading

param OPT_EGO_SPEED = Range(6, 9)
param OPT_SHARP_STEER = Range(-0.8, -0.6)

behavior EgoBehavior(speed, steer_val):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory) until (self in intersection)
    while self in intersection:
        take SetSteerAction(steer_val), SetThrottleAction(0.5)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_SHARP_STEER)

adversary = new Car at advSpawnPt,
	with blueprint MODEL

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "red")
        wait

require monitor TrafficLights()
require 15 <= (distance from egoSpawnPt to intersection) <= 25
require (distance from advSpawnPt to intersection) <= 5

terminate when (distance from ego to egoSpawnPt) > 40
terminate after 15 seconds