description = "Ego vehicle changes lanes to merge into a gap between two vehicles in the adjacent lane in preparation for a turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_AHEAD_DIST = Range(10, 30)
param OPT_BEHIND_DIST = Range(10, 30)

egoLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is None and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            egoLaneSecs.append(laneSec)

egoLaneSec = Uniform(*egoLaneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adjLaneSec = egoLaneSec._laneToRight
mergePos = adjLaneSec.centerline.project(egoSpawnPt.position)
advAheadSpawnPt = new OrientedPoint following roadDirection from mergePos for globalParameters.OPT_AHEAD_DIST
advBehindSpawnPt = new OrientedPoint following roadDirection from mergePos for -globalParameters.OPT_BEHIND_DIST

param OPT_EGO_SPEED = Range(5, 10)
param OPT_EGO_LC_START = Range(15, 40)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to egoSpawnPt > globalParameters.OPT_EGO_LC_START)
    do LaneChangeBehavior(laneSectionToSwitch=adjLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advAheadSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

behavior AdversaryBehindBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversaryBehind = new Car at advBehindSpawnPt,
	with behavior AdversaryBehindBehavior()

param TERM_DIST = Range(70, 100)

require (distance from ego to adversary) >= 10
require (distance from ego to adversaryBehind) >= 10

terminate when (distance to egoSpawnPt) > globalParameters.TERM_DIST