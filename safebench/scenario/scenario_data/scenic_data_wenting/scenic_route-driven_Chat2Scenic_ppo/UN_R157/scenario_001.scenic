description = "An adversary vehicle suddenly merges in front of the ego vehicle."

param map = localPath('../../maps/Town04.xodr')

param carla_map = 'Town04'

model scenic.simulators.carla.model

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'



param OPT_ADV_DIST = Range(10, 20)



EgoSpawnPt = globalParameters.spawnPt

yaw = globalParameters.yaw

egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw



egoLaneSec = network.laneSectionAt(egoSpawnPt)



possibleAdjSections = []

if egoLaneSec._laneToLeft is not None and egoLaneSec._laneToLeft.isForward:

    possibleAdjSections.append(egoLaneSec._laneToLeft)

if egoLaneSec._laneToRight is not None and egoLaneSec._laneToRight.isForward:

    possibleAdjSections.append(egoLaneSec._laneToRight)



adjLaneSec = Uniform(*possibleAdjSections)

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)

advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_DIST



ego = new Car at egoSpawnPt,

    with rolename 'hero',

    with blueprint MODEL



param OPT_ADV_SPEED = Range(7, 10)

param OPT_TRIGGER_DIST = Range(12, 15)



behavior AdversaryBehavior():

    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego < globalParameters.OPT_TRIGGER_DIST)

    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)

    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)



adversary = new Car at advSpawnPt,

    with blueprint MODEL,

    with regionContainedIn adjLaneSec,

    with behavior AdversaryBehavior()



require (distance from egoSpawnPt to advSpawnPt) >= 15

terminate when (distance from ego to egoSpawnPt) > 150

terminate after 30 seconds

