description = "Ego vehicle goes straight through intersection; adversarial vehicle cuts off ego from left front."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint on advInitLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(12, 15)
param ADV_TRIGGER_DIST = Range(15, 25)

behavior AdvBehavior(trajectory, target_speed, trigger_dist):
    wait until (distance from self to ego) < trigger_dist
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

advAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior(advTrajectory, globalParameters.ADV_SPEED, globalParameters.ADV_TRIGGER_DIST)

param REQ_EGO_DIST_MIN = 30
param REQ_EGO_DIST_MAX = 40
param REQ_ADV_DIST_MIN = 30
param REQ_ADV_DIST_MAX = 40

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advAgent, 100):
            setClosestTrafficLightStatus(advAgent, "green")
        wait

require monitor TrafficLights()
require globalParameters.REQ_EGO_DIST_MIN <= (distance from egoSpawnPt to intersection) <= globalParameters.REQ_EGO_DIST_MAX
require globalParameters.REQ_ADV_DIST_MIN <= (distance from advSpawnPt to intersection) <= globalParameters.REQ_ADV_DIST_MAX

terminate when (distance from ego to egoSpawnPt) > 100
terminate after 40 seconds