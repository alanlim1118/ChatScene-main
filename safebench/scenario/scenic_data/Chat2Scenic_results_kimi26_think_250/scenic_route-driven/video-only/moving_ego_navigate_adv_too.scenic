description = "Ego vehicle sideswiped by a sedan while attempting to bypass a slowing bus."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_BUS_DIST = Range(20, 40)
param OPT_TRUCK_DIST = Range(30, 50)
param OPT_ADV_DIST = Range(-5, 5)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leftLaneSec = egoLaneSec._laneToLeft
leftLaneBasePt = leftLaneSec.centerline.project(egoSpawnPt.position)

busSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_BUS_DIST
truckSpawnPt = new OrientedPoint following leftLaneSec.orientation from leftLaneBasePt for globalParameters.OPT_TRUCK_DIST
advSpawnPt = new OrientedPoint following leftLaneSec.orientation from leftLaneBasePt for globalParameters.OPT_ADV_DIST

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL

param OPT_TRUCK_SPEED = Range(5, 10)

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

truck = new Truck at truckSpawnPt,
    with heading truckSpawnPt.heading,
    with blueprint MODEL,
    with behavior TruckBehavior()

param OPT_BUS_SPEED = Range(5, 10)
param OPT_BUS_TRAVEL_TIME = Range(3, 6)

behavior BusBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED) for globalParameters.OPT_BUS_TRAVEL_TIME seconds
    while True:
        take SetBrakeAction(1)

bus = new Car at busSpawnPt,
    with heading busSpawnPt.heading,
    with regionContainedIn None,
    with behavior BusBehavior()

param OPT_ADV_SPEED = Range(8, 12)
param OPT_ADV_FWD_TIME = Range(1, 3)

behavior SedanBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for globalParameters.OPT_ADV_FWD_TIME seconds
	do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec._laneToRight, target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

sedan = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with blueprint MODEL,
	with behavior SedanBehavior()

TERM_DIST = 100

require (distance from ego to bus) > 10
require (distance from ego to truck) > 10
require (distance from ego to sedan) > 1
terminate when (ego intersects bus or ego intersects truck or ego intersects sedan or (distance to egoSpawnPt) > TERM_DIST)