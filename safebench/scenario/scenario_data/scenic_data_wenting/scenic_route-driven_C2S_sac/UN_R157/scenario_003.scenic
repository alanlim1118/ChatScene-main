description = "An adversary vehicle suddenly decelerates in front of the ego vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(15, 25)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)
param OPT_ADV_BRAKE = Range(0.7, 1.0)
param OPT_BRAKE_THRESHOLD = Range(12, 15)

behavior AdversaryBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.OPT_BRAKE_THRESHOLD:
        take SetBrakeAction(globalParameters.OPT_ADV_BRAKE)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate after 20 seconds
