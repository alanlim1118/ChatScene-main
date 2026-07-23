description = "Ego vehicle sideswiped by a sedan while attempting to bypass a slowing bus."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_BUS_DIST = Range(20, 40)
param OPT_TRUCK_DIST = Range(30, 50)
param OPT_ADV_DIST = Range(-5, 5)

laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft
leftLaneBasePt = leftLaneSec.centerline.project(egoSpawnPt.position)

busSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_BUS_DIST
truckSpawnPt = new OrientedPoint following leftLaneSec.orientation from leftLaneBasePt for globalParameters.OPT_TRUCK_DIST
advSpawnPt = new OrientedPoint following leftLaneSec.orientation from leftLaneBasePt for globalParameters.OPT_ADV_DIST

param OPT_EGO_SPEED = Range(5, 10)
param OPT_EGO_LC_DISTANCE = Range(10, 20)
param OPT_AVOID_DIST = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to busSpawnPt < globalParameters.OPT_EGO_LC_DISTANCE)
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_AVOID_DIST):
        take SetSteerAction(1)
        take SetBrakeAction(1)
        terminate

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

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