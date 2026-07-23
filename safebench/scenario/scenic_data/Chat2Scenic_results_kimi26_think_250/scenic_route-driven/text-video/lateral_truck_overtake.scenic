description = "Ego vehicle travels on a two-lane road when an overtaking truck abruptly merges back to avoid an oncoming motorcycle, causing a side-impact collision and forcing the ego to decelerate."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_TRUCK_BEHIND_DIST = Range(10, 20)
param OPT_MOTO_AHEAD_DIST = Range(50, 70)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leftLaneSec = egoLaneSec._laneToLeft

truckRefPt = new OrientedPoint behind egoSpawnPt by globalParameters.OPT_TRUCK_BEHIND_DIST
truckProjectPt = leftLaneSec.centerline.project(truckRefPt.position)
truckSpawnPt = new OrientedPoint at truckProjectPt
truckHeading = leftLaneSec.orientation[truckProjectPt]

motoRefPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD_DIST
motoProjectPt = leftLaneSec.centerline.project(motoRefPt.position)
motoSpawnPt = new OrientedPoint at motoProjectPt
motoHeading = leftLaneSec.orientation[motoProjectPt]

param OPT_EGO_SPEED = Range(7, 10)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param TRUCK_SPEED = globalParameters.OPT_EGO_SPEED + Range(3, 5)
param OPT_OVERTAKE_DIST = Range(8, 12)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to ego) > globalParameters.OPT_OVERTAKE_DIST
	do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.TRUCK_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

truck = new Truck at truckSpawnPt,
	facing egoSpawnPt.heading,
	with behavior TruckBehavior()

param OPT_MOTO_SPEED = Range(8, 14)

behavior MotoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED)

motorcycle = new Motorcycle at motoSpawnPt,
	facing motoHeading,
	with behavior MotoBehavior()

terminate when (truck intersects ego)