description = "Ego vehicle executes a continuous left lane change from the rightmost lane on a multi-lane road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoRoad = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 2, network.roads))
egoInitLane = egoRoad.forwardLanes.lanes[-1]
egoLaneSec = Uniform(*egoInitLane.sections)
leftLaneSec = egoLaneSec._laneToLeft
egoLeftLane = leftLaneSec.lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

param OPT_EGO_SPEED = Range(8, 12)
param OPT_FOLLOW_DIST = Range(15, 25)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to egoSpawnPt > globalParameters.OPT_FOLLOW_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()