description = "Ego vehicle travels straight while an adversarial vehicle in the adjacent lane below steers away toward the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*filter(lambda l: any(s._laneToRight is not None for s in l.sections), network.lanes))
egoLaneSec = Uniform(*filter(lambda s: s._laneToRight is not None, egoLane.sections))
advLaneSec = egoLaneSec._laneToRight
advLane = advLaneSec.lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(8, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(8, 10)

behavior AdversaryBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=advLaneSec._laneToRight, target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()