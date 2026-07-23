description = "Ego vehicle passes an oncoming vehicle on a straight two-lane road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

twoLaneRoads = filter(lambda r: len(r.lanes) == 2 and r.forwardLanes is not None and r.backwardLanes is not None, network.roads)
selectedRoad = Uniform(*twoLaneRoads)
egoLane = selectedRoad.forwardLanes.lanes[0]
advLane = selectedRoad.backwardLanes.lanes[0]
egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST_MIN = 20
INIT_DIST_MAX = 60
TERM_DIST = 100

require INIT_DIST_MIN <= (distance from ego to adversary) <= INIT_DIST_MAX
terminate when (distance from ego to adversary) > TERM_DIST