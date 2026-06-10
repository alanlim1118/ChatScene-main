description = "Ego vehicle attempts a lane change to avoid a slow vehicle, but an adversary in the target lane blocks it by slowing down."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(15, 25)
param OPT_ADV_DIST = Range(-5, 10)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
slowCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_DIST

egoTrajectory = [egoLaneSec.lane, adjLaneSec.lane]
slowCarTrajectory = [egoLaneSec.lane]
advTrajectory = [adjLaneSec.lane]

param OPT_EGO_SPEED = Range(10, 15)
param OPT_LC_DIST = Range(10, 20)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.OPT_LC_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=adjLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_LEADING_SPEED = globalParameters.OPT_EGO_SPEED - 5

slowCar = new Car at slowCarSpawnPt,
    with blueprint MODEL,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED + 5
param OPT_ADV_GAP_THRESHOLD = 15

behavior AdvBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when (distance from self to slowCar) < globalParameters.OPT_ADV_GAP_THRESHOLD:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

advAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(slowCar, 100):
            setClosestTrafficLightStatus(slowCar, "green")
        if withinDistanceToTrafficLight(advAgent, 100):
            setClosestTrafficLightStatus(advAgent, "green")
        wait

require monitor TrafficLights()

terminate when (distance from ego to egoSpawnPt) > 150
terminate after 60 seconds