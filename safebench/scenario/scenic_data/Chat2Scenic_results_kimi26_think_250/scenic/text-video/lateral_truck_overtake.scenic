description = "Ego vehicle travels on a two-lane road when an overtaking truck abruptly merges back to avoid an oncoming motorcycle, causing a side-impact collision and forcing the ego to decelerate."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_TRUCK_BEHIND_DIST = Range(10, 20)
param OPT_MOTO_AHEAD_DIST = Range(50, 70)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft

truckRefPt = new OrientedPoint behind egoSpawnPt by globalParameters.OPT_TRUCK_BEHIND_DIST
truckProjectPt = leftLaneSec.centerline.project(truckRefPt.position)
truckSpawnPt = new OrientedPoint at truckProjectPt
truckHeading = leftLaneSec.orientation[truckProjectPt]

motoRefPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD_DIST
motoProjectPt = leftLaneSec.centerline.project(motoRefPt.position)
motoSpawnPt = new OrientedPoint at motoProjectPt
motoHeading = leftLaneSec.orientation[motoProjectPt]

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param TRUCK_SPEED = globalParameters.EGO_SPEED + Range(3, 5)
param OVERTAKE_DIST = Range(8, 12)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to ego) > globalParameters.OVERTAKE_DIST
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