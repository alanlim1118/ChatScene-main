description = "Ego vehicle exits a parallel parking bay into traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_START_DIST = Range(10, 20) * -1

outerLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is None:
            outerLaneSecs.append(laneSec)

roadLaneSec = Uniform(*outerLaneSecs)
lanePt = new OrientedPoint in roadLaneSec.centerline

egoSpawnPt = new OrientedPoint right of lanePt by 3, facing lanePt.heading
advSpawnPt = new OrientedPoint following roadDirection from lanePt for globalParameters.OPT_ADV_START_DIST

param OPT_EGO_SPEED = Range(5, 8)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=roadLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()