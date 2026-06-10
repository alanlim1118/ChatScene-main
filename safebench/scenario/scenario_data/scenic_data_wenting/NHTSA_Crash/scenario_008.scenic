description = "Vehicle A starts to proceed at a crossroad, stops abruptly, and is rear-ended by Vehicle B who misinterprets A's initial movement."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(5, 10)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_WAIT_TIME = Range(1, 3)
param OPT_ADV_DRIVE_TIME = Range(1.5, 3)
param OPT_ADV_SPEED = Range(5, 8)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior():
    do WaitBehavior() for globalParameters.OPT_ADV_WAIT_TIME seconds
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for globalParameters.OPT_ADV_DRIVE_TIME seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

AdvAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 10

TERM_DIST = 150
terminate when ego intersects AdvAgent
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
