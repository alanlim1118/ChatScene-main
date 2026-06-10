description = "Vehicle A starts to proceed at a crossroad, stops abruptly, and is rear-ended by Vehicle B who misinterprets A's initial movement."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.connectingLane is not None, intersection.maneuvers))
advSpawnPt = new OrientedPoint on egoManeuver.connectingLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = egoTrajectory

param EGO_SPEED = Range(6, 10)
param EGO_WAIT_DIST = 10
param EGO_CROSS_DIST = 20

behavior EgoBehavior(trajectory, speed, wait_dist, cross_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        egoTrajectory,
        globalParameters.EGO_SPEED,
        globalParameters.EGO_WAIT_DIST,
        globalParameters.EGO_CROSS_DIST
    )

param ADV_WAIT_TIME = Range(1, 3)
param ADV_DRIVE_TIME = Range(1.5, 3)
param ADV_SPEED = Range(5, 8)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior():
    do WaitBehavior() for globalParameters.ADV_WAIT_TIME seconds
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory) for globalParameters.ADV_DRIVE_TIME seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 10
terminate when ego intersects AdvAgent