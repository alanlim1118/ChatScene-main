description = "Target vehicle merges into ego vehicle's path with a full lateral displacement."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoSection = network.laneSectionAt(egoSpawnPt)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 9)
param OPT_ADV_DISTANCE = Range(10, 15)

behavior AdvBehavior(speed, merge_dist):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < merge_dist
    do LaneChangeBehavior(laneSectionToSwitch=egoSection, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

AdvAgent = new Car ahead of egoSpawnPt by Range(20, 30),
    facing egoSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE)

require ego can see AdvAgent
terminate when (distance from ego to egoSpawnPt) > 150
terminate after 60 seconds
