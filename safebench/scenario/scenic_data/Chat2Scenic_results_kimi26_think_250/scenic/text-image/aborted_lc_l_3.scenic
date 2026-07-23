description = "Ego vehicle attempts an aborted lane change to the left while two adversarial vehicles block the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_AHEAD_DIST = Range(15, 25)
param OPT_BEHIND_DIST = Range(15, 25)

laneSecsCenter = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsCenter.append(laneSec)

egoLaneSec = Uniform(*laneSecsCenter)
adjLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
mergeBase = adjLaneSec.centerline.project(egoSpawnPt.position)
advAheadSpawnPt = new OrientedPoint following roadDirection from mergeBase for globalParameters.OPT_AHEAD_DIST
advBehindSpawnPt = new OrientedPoint following roadDirection from mergeBase for -globalParameters.OPT_BEHIND_DIST

advAhead = new Car at advAheadSpawnPt,
    with heading advAheadSpawnPt.heading,
    with regionContainedIn roadOrShoulder,
    with behavior FollowLaneBehavior(10)

advBehind = new Car at advBehindSpawnPt,
    with heading advBehindSpawnPt.heading,
    with regionContainedIn roadOrShoulder,
    with behavior FollowLaneBehavior(10)