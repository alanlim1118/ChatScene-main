description = "Ego vehicle collides head-on with an oncoming vehicle encroaching into its lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None, network.roads))
egoLane = Uniform(*road.forwardLanes.lanes)
advLane = Uniform(*road.backwardLanes.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, laneToFollow=egoLane, is_oppositeTraffic=True)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

terminate when ego intersects adversary