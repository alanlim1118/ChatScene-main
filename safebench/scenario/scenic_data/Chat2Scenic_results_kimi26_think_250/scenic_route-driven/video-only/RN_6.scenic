description = "Ego vehicle approaching a roundabout is cut off by an adversary merging from the left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToLeft

advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 14)
param OPT_ADV_CUTIN_DIST = Range(15, 25)

behavior AdvBehavior(adv_speed, cutin_dist):
    do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego < cutin_dist)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=adv_speed)
    do FollowLaneBehavior(target_speed=adv_speed)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior AdvBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_ADV_CUTIN_DIST
    )