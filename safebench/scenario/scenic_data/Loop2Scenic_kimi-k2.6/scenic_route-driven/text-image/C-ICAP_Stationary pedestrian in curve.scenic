description = "The ego approaches a parked car that is blocking its lane and must use the opposite lane to bypass the vehicle, cautiously monitoring oncoming traffic, and suddenly encounters a jaywalking pedestrian, requiring the ego to quickly assess the situation and respond appropriately to avoid a collision."

Town = 'Town01'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 30)
param OPT_GEO_X_DISTANCE = Range(-1, 1)
param OPT_GEO_Y_DISTANCE = Range(2, 6)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

IntSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
SHIFT = globalParameters.OPT_GEO_X_DISTANCE @ globalParameters.OPT_GEO_Y_DISTANCE

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

Blocker = new Car at IntSpawnPt,
	with heading IntSpawnPt.heading,
	with regionContainedIn None

param OPT_ADV_SPEED = Range(1, 2)
param OPT_ADV_DISTANCE = Range(8, 12)

behavior AdvBehavior(adv_speed, adv_distance):
	do CrossingBehavior(ego, adv_speed, adv_distance)

AdvAgent = new Pedestrian at Blocker offset along IntSpawnPt.heading by SHIFT,
	with heading IntSpawnPt.heading + 90 deg,
	with regionContainedIn None,
	with behavior AdvBehavior(
		globalParameters.OPT_ADV_SPEED,
		globalParameters.OPT_ADV_DISTANCE
	)

require distance to intersection >= 100  # Ensure the ego vehicle is far from the intersection

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
