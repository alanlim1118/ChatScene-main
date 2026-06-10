description = "Ego vehicle maneuvers to allow an emergency vehicle to pass from behind."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

advSpawnPt = new OrientedPoint behind egoSpawnPt by Range(15, 25)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(12, 15)

behavior AdvBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

advVehicle = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advVehicle, 100):
            setClosestTrafficLightStatus(advVehicle, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from advSpawnPt to egoSpawnPt) <= 25

terminate when (distance from advVehicle to egoSpawnPt) > 100
terminate after 60 seconds
