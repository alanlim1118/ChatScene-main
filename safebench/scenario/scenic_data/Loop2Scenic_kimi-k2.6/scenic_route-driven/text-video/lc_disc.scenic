description = "The ego vehicle is trying to change to the left lane; the adversarial car is driving parallel to the ego and blocking its maneuver."

Town = 'Town03'
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)

adjLaneSec = egoLaneSec._laneToLeft
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn egoLaneSec,
	with blueprint EGO_MODEL

param OPT_EGO_SPEED = Range(1, 5)  # Speed for the ego vehicle

AdvAgent = new Car at adjLanePt,
	with heading egoSpawnPt.heading,
	with regionContainedIn adjLaneSec,
	with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

require distance to intersection >= 100

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
