description = "Ego vehicle overtaking a cargo truck near an underpass is struck when the truck drifts left into its lane."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToRight
advInitLane = advLaneSec.lane
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param OPT_OVERTAKE_DIST = Range(10, 20)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego < globalParameters.OPT_OVERTAKE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Truck at advSpawnPt,
    with regionContainedIn advLaneSec,
    with behavior TruckBehavior()

terminate when (ego intersects adversary)