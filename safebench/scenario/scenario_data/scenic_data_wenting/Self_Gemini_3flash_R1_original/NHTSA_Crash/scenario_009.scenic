description = "Vehicle B rear-ended Vehicle A while braking on a slick road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(15, 25)

param OPT_EGO_SPEED = Range(18, 22)
param OPT_BRAKE_DISTANCE = Range(8, 12)

behavior EgoBehavior(speed, brake_dist):
        do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

param OPT_ADV_SPEED = Range(15, 18)
param OPT_ADV_BRAKE_DELAY = Range(1, 3)

behavior AdvBehavior(speed, delay):
    do FollowLaneBehavior(target_speed=speed) for delay seconds
    while True:
        take SetBrakeAction(1)
        take SetThrottleAction(0)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_BRAKE_DELAY)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects adv