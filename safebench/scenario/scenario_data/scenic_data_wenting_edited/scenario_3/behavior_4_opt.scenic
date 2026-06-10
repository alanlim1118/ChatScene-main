description = "Ego vehicle attempts a lane change to evade a slow leader, complicated by a weaving adversarial vehicle in the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(15, 25)
param OPT_ADV_OFFSET = Range(-10, 5)

laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeft.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeft)
targetLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

targetLaneRefPt = targetLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from targetLaneRefPt for globalParameters.OPT_ADV_OFFSET

param EGO_SPEED = Range(12, 15)
param OVERTAKE_THRESHOLD = 20

behavior EgoBehavior(target_speed, overtake_dist, target_lane):
    try:
        do FollowLaneBehavior(target_speed=target_speed) until withinDistanceToObjsInLane(self, overtake_dist)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.OVERTAKE_THRESHOLD, targetLaneSec)

param OPT_LEAD_SPEED = globalParameters.EGO_SPEED - 5

leadVehicle = new Car at leadSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

param WEAVE_SPEED = Range(10, 13)
param WEAVE_AMPLITUDE = Range(2.0, 3.0)
param WEAVE_PERIOD = Range(15, 20)
param WEAVE_THRESHOLD = 40

behavior WeavePIDBehavior(target_speed, weave_amplitude, weave_period):
    K_P = 0.2
    K_D = 0.1
    K_I = 0.01
    dt = 0.1
    pid = PIDLateralController(K_P, K_D, K_I, dt)
    pid.windup_guard = 0.5
    past_steer = 0.0
    while True:
        trajectoryLine = self.laneSection.centerline
        proj = trajectoryLine.project(self.position)
        progress = distance from trajectoryLine[0] to proj
        sine_offset = weave_amplitude * sin(progress / weave_period)
        cte = trajectoryLine.signedDistanceTo(self.position) - sine_offset
        steer = pid.run_step(cte)
        take RegulatedControlAction(target_speed, steer, past_steer)
        past_steer = steer
        wait

behavior AdversaryWeaveBehavior(speed, amplitude, period, threshold):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < threshold
    do WeavePIDBehavior(speed, amplitude, period)

weavingAdv = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdversaryWeaveBehavior(globalParameters.WEAVE_SPEED, globalParameters.WEAVE_AMPLITUDE, globalParameters.WEAVE_PERIOD, globalParameters.WEAVE_THRESHOLD)

param MAX_TIME = 60
param TERM_DIST = 200

monitor TrafficLightControl():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(leadVehicle, 100):
            setClosestTrafficLightStatus(leadVehicle, "green")
        if withinDistanceToTrafficLight(weavingAdv, 100):
            setClosestTrafficLightStatus(weavingAdv, "green")
        wait

require monitor TrafficLightControl()
require ego can see leadVehicle
require (distance from ego to weavingAdv) > 5

terminate when (distance from ego to egoSpawnPt) > globalParameters.TERM_DIST
terminate after globalParameters.MAX_TIME seconds