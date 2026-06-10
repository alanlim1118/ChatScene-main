description = "Vehicle turning at an intersection loses control on a wet road and runs off the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

intersection = Uniform(*network.intersections)

egoManeuver = Uniform(*filter(lambda m: m.type in {ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN}, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

param EGO_SPEED = Range(7, 10)
param LOSS_CONTROL_STEER = Uniform(Range(-0.9, -0.6), Range(0.6, 0.9))
param LOSS_CONTROL_THROTTLE = Range(0.8, 1.0)

behavior EgoBehavior(trajectory, steer_val, throttle_val):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory) for Range(2, 4) seconds
    finally:
        while True:
            take SetSteerAction(steer_val), SetThrottleAction(throttle_val)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.LOSS_CONTROL_STEER, globalParameters.LOSS_CONTROL_THROTTLE)

monitor TrafficLightManager():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightManager()
require 20 <= (distance from egoSpawnPt to intersection) <= 40
terminate when (ego not in network.roads) and (distance from ego to egoSpawnPt > 10)