description = "Ego vehicle overtaking a cargo truck near an underpass is struck when the truck drifts left into its lane."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 2, network.roads))
egoInitLane = Uniform(*road.forwardLanes.lanes[:-1])
egoLaneSec = Uniform(*filter(lambda s: s._laneToRight is not None, egoInitLane.sections))
advLaneSec = egoLaneSec._laneToRight
advInitLane = advLaneSec.lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param OPT_EGO_SPEED = Range(8, 12)
param OPT_OVERTAKE_DIST = Range(10, 20)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to advSpawnPt < globalParameters.OPT_OVERTAKE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=advLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until (distance from self to ego < globalParameters.OPT_OVERTAKE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.ADV_SPEED)

adversary = new Truck at advSpawnPt,
    with regionContainedIn advLaneSec,
    with behavior TruckBehavior()

terminate when (ego intersects adversary)