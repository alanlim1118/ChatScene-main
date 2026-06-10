description = "Vehicle B rear-ends Vehicle A due to tailgating and sudden stop."
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
terminate when ego intersects adv