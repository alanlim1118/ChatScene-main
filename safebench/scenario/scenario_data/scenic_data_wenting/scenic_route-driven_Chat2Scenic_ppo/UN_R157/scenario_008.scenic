description = "Ego vehicle avoids collision with an emergency-braking leading vehicle on a highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE_BEHIND = Range(20, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

egoInitLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE_BEHIND

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 20)
param OPT_ADV_DRIVE_TIME = Range(4, 8)

behavior AdversarialBehavior(speed, drive_time):
    do FollowLaneBehavior(target_speed=speed) for drive_time seconds
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversarialBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DRIVE_TIME)

TERM_DIST = 100

require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance from ego to egoSpawnPt) > TERM_DIST