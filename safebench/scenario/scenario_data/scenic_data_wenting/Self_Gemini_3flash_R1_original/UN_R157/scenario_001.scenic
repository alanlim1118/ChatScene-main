description = "An adversary vehicle suddenly merges in front of the ego vehicle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(10, 20)

validPairs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward:
            if sec._laneToLeft and sec._laneToLeft.isForward:
                validPairs.append((sec, sec._laneToLeft))
            if sec._laneToRight and sec._laneToRight.isForward:
                validPairs.append((sec, sec._laneToRight))

selectedPair = Uniform(*validPairs)
egoLaneSec = selectedPair[0]
adjLaneSec = selectedPair[1]

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_DIST

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(7, 10)
param TRIGGER_DIST = Range(12, 15)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until (distance from self to ego < globalParameters.TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn adjLaneSec,
    with behavior AdversaryBehavior()

require (distance from egoSpawnPt to advSpawnPt) >= 15
terminate when (distance from ego to egoSpawnPt) > 150
terminate after 30 seconds