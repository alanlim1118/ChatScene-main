description = "Ego vehicle reacts to a suddenly stopping lead vehicle ahead, resulting in a minor collision during an evasive maneuver near an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
advInitLane = egoInitLane
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(10, 20)
suvSpawnPt = new OrientedPoint right of advSpawnPt by Range(4, 8), facing advSpawnPt.heading

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(6, 9)
param OPT_ADV_BRAKE = Range(0.8, 1.0)
param OPT_ADV_STEER = Range(0.6, 1.0)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for Range(3, 5) seconds
    while True:
        take SetBrakeAction(globalParameters.OPT_ADV_BRAKE)
        take SetSteerAction(globalParameters.OPT_ADV_STEER)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()