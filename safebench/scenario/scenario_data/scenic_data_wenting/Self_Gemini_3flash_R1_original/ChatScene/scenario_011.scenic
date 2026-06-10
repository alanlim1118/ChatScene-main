description = "Ego vehicle attempts a lane change to avoid a slow vehicle, but an adversary blocks its original lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param DIST_EGO_TO_ADV1 = Range(20, 30)
param DIST_ADV2_OFFSET = Range(-5, 5)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adv1SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.DIST_EGO_TO_ADV1

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
adv2SpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.DIST_ADV2_OFFSET

egoTrajectory = [egoLaneSec.lane]
adv1Trajectory = [egoLaneSec.lane]
adv2Trajectory = [adjLaneSec.lane, egoLaneSec.lane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed, target_lane):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, 25)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, adjLaneSec)

param ADV1_SPEED = Range(3, 5)

behavior Adv1Behavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adv1 = new Car at adv1SpawnPt,
    with heading adv1SpawnPt.heading,
    with blueprint MODEL,
    with behavior Adv1Behavior(globalParameters.ADV1_SPEED)

param ADV2_SPEED = Range(12, 15)

behavior Adv2Behavior(speed, target_lane):
    do FollowLaneBehavior(target_speed=speed) until (distance from ego to adv1) < 30
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adv2 = new Car at adv2SpawnPt,
    with heading adv2SpawnPt.heading,
    with blueprint MODEL,
    with behavior Adv2Behavior(globalParameters.ADV2_SPEED, egoLaneSec)

param TERMINATE_DIST = 100

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv1, 100):
            setClosestTrafficLightStatus(adv1, "green")
        if withinDistanceToTrafficLight(adv2, 100):
            setClosestTrafficLightStatus(adv2, "green")
        wait

require monitor TrafficLights()
require ego can see adv1
require ego can see adv2

terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATE_DIST
terminate after 60 seconds