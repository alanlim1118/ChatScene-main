description = "Vehicle B rear-ended Vehicle A while braking on a slick road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 25)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 18)
param OPT_ADV_BRAKE_DELAY = Range(1, 3)

behavior AdvBehavior(speed, delay):
    do FollowLaneBehavior(target_speed=speed) for delay seconds
    while True:
        take SetBrakeAction(1)
        take SetThrottleAction(0)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_BRAKE_DELAY)

TERM_DIST = 150

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects adv
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
