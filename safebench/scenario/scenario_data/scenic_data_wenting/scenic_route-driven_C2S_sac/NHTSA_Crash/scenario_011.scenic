description = "Vehicle A changes lanes into Vehicle B's path, forcing Vehicle B to brake and steer to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToRight
require advLaneSec is not None
require advLaneSec.isForward
require advLaneSec.group and len(advLaneSec.group.lanes) == 2
require advLaneSec._laneToRight is None and advLaneSec._laneToLeft is not None

advSpawnPt = new OrientedPoint in advLaneSec.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)
param OPT_ADV_BRAKE = Range(0.7, 0.9)
param OPT_ADV_STEER = Range(0.2, 0.4)
param ADV_THRESHOLD = 15

behavior AdvBehavior(speed, brake_val, steer_val, threshold):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=threshold):
        take SetBrakeAction(brake_val), SetSteerAction(steer_val)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_BRAKE, globalParameters.OPT_ADV_STEER, globalParameters.ADV_THRESHOLD)

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
