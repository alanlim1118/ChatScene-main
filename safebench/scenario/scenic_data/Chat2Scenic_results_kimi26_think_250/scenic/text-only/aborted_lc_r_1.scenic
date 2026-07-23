description = "Ego vehicle performs an aborted lane change to the right while an adversarial vehicle proceeds straight in the adjacent right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)
egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
egoInitLane = egoLaneSec.lane
advLaneSec = egoLaneSec._laneToRight
advInitLane = advLaneSec.lane
advSpawnPt = new OrientedPoint at advLaneSec.centerline.project(egoSpawnPt.position) facing egoSpawnPt.heading

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ABORT_DURATION = Range(1.5, 2.5)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToRight, target_speed=globalParameters.OPT_EGO_SPEED) for globalParameters.OPT_ABORT_DURATION
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(5, 10)

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn advLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)