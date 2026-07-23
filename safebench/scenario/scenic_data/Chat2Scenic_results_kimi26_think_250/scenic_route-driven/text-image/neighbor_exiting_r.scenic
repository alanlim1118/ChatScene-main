description = "Ego vehicle travels straight while an adversarial vehicle in the adjacent lane below steers away toward the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
egoLane = egoLaneSec.lane
advLaneSec = egoLaneSec._laneToRight
advLane = advLaneSec.lane
advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 10)

behavior AdversaryBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=advLaneSec._laneToRight, target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()