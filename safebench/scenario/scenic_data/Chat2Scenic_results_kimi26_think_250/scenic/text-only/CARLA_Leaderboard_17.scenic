description = "Ego vehicle encounters oncoming traffic invading its lane on a bend due to an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda lane:
    all([sec._laneToLeft is not None and sec._laneToLeft.isForward is not sec.isForward for sec in lane.sections]),
    network.lanes))

egoSpawnPt = new OrientedPoint on initLane.centerline

egoLaneSec = Uniform(*initLane.sections)
advLaneSec = egoLaneSec._laneToLeft

advSpawnPt = new OrientedPoint on advLaneSec.centerline

obsSpawnPt = new OrientedPoint following advLaneSec.orientation from advSpawnPt for Range(5, 15)

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BRAKE_DIST = Range(8, 12)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)
param ADV_SWERVE_TIME = Range(0.5, 1.5)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for globalParameters.ADV_SWERVE_TIME seconds
    do LaneChangeBehavior(egoLaneSec, True, globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, is_oppositeTraffic=True)

adversary = new Car at advSpawnPt,
    facing advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior AdvBehavior()

debris = new Debris at obsSpawnPt,
    with regionContainedIn None

EGO_ADV_MIN_DIST = 20
EGO_ADV_MAX_DIST = 50
EGO_OBS_MIN_DIST = 25
EGO_OBS_MAX_DIST = 60
TERM_DIST = 100

require EGO_ADV_MIN_DIST <= (distance from ego to adversary) <= EGO_ADV_MAX_DIST
require EGO_OBS_MIN_DIST <= (distance from ego to debris) <= EGO_OBS_MAX_DIST
terminate when (distance from egoSpawnPt to ego) > TERM_DIST