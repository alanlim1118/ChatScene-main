description = "Vehicle A changes lanes into Vehicle B's path, forcing Vehicle B to brake and steer to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

curbLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec.group and len(sec.group.lanes) == 2:
            if sec._laneToRight is None and sec._laneToLeft is not None:
                curbLaneSections.append(sec)

advLaneSec = Uniform(*curbLaneSections)
egoLaneSec = advLaneSec._laneToLeft

advSpawnPt = new OrientedPoint in advLaneSec.centerline

alignedEgoPos = egoLaneSec.centerline.project(advSpawnPt.position)
egoSpawnPt = new OrientedPoint at alignedEgoPos, facing advSpawnPt.heading

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(target_speed, target_lane):
    do FollowLaneBehavior(target_speed=target_speed) for Range(1, 3) seconds
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, advLaneSec)

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = Range(0.7, 0.9)
param ADV_STEER = Range(0.2, 0.4)
param ADV_THRESHOLD = 15

behavior AdvBehavior(speed, brake_val, steer_val, threshold):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=threshold):
        take SetBrakeAction(brake_val), SetSteerAction(steer_val)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.ADV_SPEED, globalParameters.ADV_BRAKE, globalParameters.ADV_STEER, globalParameters.ADV_THRESHOLD)

param TERMINATION_DIST = 120
param TERMINATION_TIME = 20

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLightMonitor()
require (distance from egoSpawnPt to advSpawnPt) < 10

terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATION_DIST
terminate after globalParameters.TERMINATION_TIME seconds