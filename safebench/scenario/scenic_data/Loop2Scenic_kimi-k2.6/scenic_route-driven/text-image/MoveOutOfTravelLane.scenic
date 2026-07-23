description = "The ego approaches a parked car obstructing its lane and must use the opposite lane to go around when an oncoming car suddenly turns into the ego's target lane without signaling, requiring the ego to wait untill the oncoming car has passed."

Town = 'Town01'
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 30)
param OPT_MOTO_START_DIST = Range(40, 60)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)

IntSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE

carStartPt = new OrientedPoint following roadDirection from IntSpawnPt for globalParameters.OPT_MOTO_START_DIST

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

Blocker = new Car at IntSpawnPt,
	with heading IntSpawnPt.heading,
	with regionContainedIn None

param OPT_ADV_SPEED = Range(4, 8)
param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 30)

behavior AdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, is_oppositeTraffic=True) until (distance from self to Blocker < globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE)
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

AdvAgent = new Car at carStartPt,
	with heading IntSpawnPt.heading + 180 deg,  # Facing toward ego
	with regionContainedIn egoLaneSec,
	with behavior AdvBehavior()

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
