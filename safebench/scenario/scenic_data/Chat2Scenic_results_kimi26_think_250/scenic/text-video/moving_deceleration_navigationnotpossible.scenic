description = "Ego vehicle rear-ends a white SUV that brakes suddenly on a multi-lane highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(20, 30)
param OPT_TRUCK_DIST = Range(5, 15)
param OPT_OTHER_CAR_DIST = Range(25, 35)

laneSecsLeftmost = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is None and laneSec._laneToRight is not None:
            laneSecsLeftmost.append(laneSec)

egoLaneSec = Uniform(*laneSecsLeftmost)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST

rightLaneSec = egoLaneSec._laneToRight
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_TRUCK_DIST
otherCarSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_OTHER_CAR_DIST

param EGO_SPEED = Range(7, 10)
EGO_BRAKE = 1.0
SAFE_DIST = 10

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, SAFE_DIST):
        take SetBrakeAction(EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(5, 8)
param OPT_ADV_BRAKE_DIST = Range(10, 15)

behavior LockBrakeBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.OPT_ADV_BRAKE_DIST:
        while True:
            take SetBrakeAction(1.0)

adversarial = new Car at advSpawnPt,
    facing advSpawnPt.heading,
    with behavior LockBrakeBehavior()

param OPT_TRUCK_SPEED = Range(5, 8)

behavior TruckForwardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

truck = new Truck at truckSpawnPt,
    facing truckSpawnPt.heading,
    with behavior TruckForwardBehavior()

param OPT_OTHER_CAR_SPEED = Range(7, 10)

behavior OtherCarForwardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_OTHER_CAR_SPEED)

otherCar = new Car at otherCarSpawnPt,
    facing otherCarSpawnPt.heading,
    with behavior OtherCarForwardBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	facing advSpawnPt.heading,
	with behavior AdversaryBehavior()

require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (ego intersects adversarial)