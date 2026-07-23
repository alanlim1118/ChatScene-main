description = "Ego vehicle following a box truck on a multi-lane city road is struck by a black sedan cutting sharply from the right to avoid a stationary vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_TRUCK_DIST = Range(10, 15)
param OPT_STATIONARY_AHEAD_TRUCK = Range(20, 30)
param OPT_ADV_BEHIND_STATIONARY = Range(5, 15) * -1

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

truckSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_TRUCK_DIST

rightLaneSec = egoLaneSec._laneToRight

stationaryTemp = new OrientedPoint following roadDirection from truckSpawnPt for globalParameters.OPT_STATIONARY_AHEAD_TRUCK
stationarySpawnPt = rightLaneSec.centerline.project(stationaryTemp.position)

advTemp = new OrientedPoint following roadDirection from stationaryTemp for globalParameters.OPT_ADV_BEHIND_STATIONARY
advSpawnPt = rightLaneSec.centerline.project(advTemp.position)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param TRUCK_SPEED = Range(5, 10)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

truck = new Truck at truckSpawnPt,
	with behavior TruckBehavior()

param ADV_SPEED = Range(8, 12)
param ADV_AVOID_DIST = Range(6, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until (distance from self to stationarySpawnPt) < globalParameters.ADV_AVOID_DIST
    leftLaneSec = self.laneSection.laneToLeft
    do LaneChangeBehavior(
            laneSectionToSwitch=leftLaneSec,
            target_speed=globalParameters.ADV_SPEED)
    take SetBrakeAction(1.0)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

stationary = new Car at stationarySpawnPt

terminate when (ego intersects adversary)