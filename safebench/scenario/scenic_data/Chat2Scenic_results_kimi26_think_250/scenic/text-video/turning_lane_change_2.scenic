description = "Ego vehicle travels on a busy urban road when a white sedan aggressively cuts across traffic, causing a side-impact collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is None and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight
advLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advLanePt for 0

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    do LaneChangeBehavior(egoLaneSec, False, globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior AdvBehavior()