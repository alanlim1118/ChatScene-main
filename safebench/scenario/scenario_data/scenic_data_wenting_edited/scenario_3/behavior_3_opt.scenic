description = "Ego vehicle lane changes to evade a slow vehicle, but an adversarial car in the target lane suddenly brakes, requiring quick ego reaction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 15)
param OPT_ADV_DIST = Range(20, 30)

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
adjLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
slowCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advCarSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_DIST

egoTrajectory = [egoLaneSec.lane]
slowCarTrajectory = [egoLaneSec.lane]
advCarTrajectory = [adjLaneSec.lane]

param EGO_SPEED = Range(12, 15)
param BRAKE_THRESHOLD = 10
param CHANGE_THRESHOLD = 20

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.CHANGE_THRESHOLD)
        do LaneChangeBehavior(laneSectionToSwitch=adjLaneSec, target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.BRAKE_THRESHOLD):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_SLOW_SPEED = Range(5, 7)

slowCar = new Car at slowCarSpawnPt,
    with blueprint MODEL,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_SLOW_SPEED)

param OPT_ADV_SPEED = Range(10, 12)
param OPT_BRAKE_TRIGGER_DIST = Range(8, 15)

behavior AdvBehavior(target_speed, trigger_dist):
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego) < trigger_dist
    while True:
        take SetBrakeAction(1.0)

advCar = new Car at advCarSpawnPt,
    with blueprint MODEL,
    with regionContainedIn adjLaneSec,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_BRAKE_TRIGGER_DIST)

terminate when (ego intersects slowCar) or (ego intersects advCar)
terminate when (distance from ego to advCar) > 50