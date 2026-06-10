description = "Vehicle B rear-ends Vehicle A due to tailgating and sudden stop."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
advInitLane = egoInitLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)

param OPT_EGO_SPEED = Range(12, 15)
param OPT_BRAKE_THRESHOLD = Range(5, 8)

behavior EgoBehavior(target_speed, brake_threshold):
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

param OPT_ADV_SPEED = Range(12, 15)
param OPT_ADV_DRIVE_TIME = Range(3, 5)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior(target_speed, drive_time):
    do FollowLaneBehavior(target_speed=target_speed) for drive_time seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DRIVE_TIME)

ADV_EGO_DIST = [5, 10]
require ADV_EGO_DIST[0] <= (distance from egoSpawnPt to advSpawnPt) <= ADV_EGO_DIST[1]
#terminate when ego intersects adv