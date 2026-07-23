description = "Ego vehicle travels straight on a two-lane road approaching a lateral adversary crossing into its path from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_CROSS_DIST = Range(20, 40)
param OPT_ADV_LATERAL = Range(4, 8)

laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

crossPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_CROSS_DIST
advSpawnPt = new OrientedPoint right of crossPt by globalParameters.OPT_ADV_LATERAL, facing toward crossPt

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdversaryBehavior():
	do CrossingBehavior(reference_actor=ego, min_speed=1, threshold=50)

adversary = new Pedestrian at advSpawnPt,
	facing toward crossPt,
	with regionContainedIn None,
	with behavior AdversaryBehavior()