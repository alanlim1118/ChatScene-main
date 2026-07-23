description = "Ego vehicle travels straight in the middle lane as an adversarial object enters from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

rightLaneSec = egoLaneSec._laneToRight
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoLaneSec.orientation from rightLanePt for Range(15, 30)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 10)

behavior AdversaryBehavior():
	do LaneChangeBehavior(egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()