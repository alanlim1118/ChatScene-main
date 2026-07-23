description = "Ego vehicle travels straight on a two-lane road approaching a lateral adversary crossing into its path from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_CROSS_DIST = Range(20, 40)
param OPT_ADV_LATERAL = Range(4, 8)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

crossPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_CROSS_DIST
advSpawnPt = new OrientedPoint right of crossPt by globalParameters.OPT_ADV_LATERAL, facing toward crossPt

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

behavior AdversaryBehavior():
	do CrossingBehavior(reference_actor=ego, min_speed=1, threshold=50)

adversary = new Pedestrian at advSpawnPt,
	facing toward crossPt,
	with regionContainedIn None,
	with behavior AdversaryBehavior()