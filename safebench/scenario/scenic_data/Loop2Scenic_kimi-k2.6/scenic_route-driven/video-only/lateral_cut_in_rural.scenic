description = "The ego encounters a parked car blocking its lane and must use the opposite lane to bypass the vehicle when an oncoming pedestrian enters the lane without warning and suddenly stops, necessitating the ego to brake sharply."

Town = 'Town01'
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(30, 40)
OPT_SIDEWALK_OFFSET = 6

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and (m.intersection.is4Way or m.intersection.is3Way), egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoLaneSec = network.laneSectionAt(egoSpawnPt)

IntSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE

PedSpawnPt = new OrientedPoint left of IntSpawnPt by OPT_SIDEWALK_OFFSET,
	with heading IntSpawnPt.heading + 90 deg

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

Blocker = new Car at IntSpawnPt,
	with heading IntSpawnPt.heading,
	with regionContainedIn None

param OPT_ADV_SPEED = Range(1, 3)
param OPT_ADV_DISTANCE = Range(8, 10)
OPT_STOP_DISTANCE = 1

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
	do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
	take SetWalkingSpeedAction(0)

AdvAgent = new Pedestrian at PedSpawnPt,
	with heading IntSpawnPt.heading - 90 deg,
	with regionContainedIn None,
	with behavior CrossAndStopBehavior(
		ego,
		globalParameters.OPT_ADV_SPEED,
		globalParameters.OPT_ADV_DISTANCE,
		egoLaneSec._laneToLeft.centerline,
		OPT_STOP_DISTANCE
	)

require (distance from egoSpawnPt to intersection) >= 200

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
