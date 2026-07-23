description = "Ego vehicle encounters oncoming traffic invading its lane on a bend due to an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToLeft

advSpawnPt = new OrientedPoint on advLaneSec.centerline

obsSpawnPt = new OrientedPoint following advLaneSec.orientation from advSpawnPt for Range(5, 15)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)
param OPT_ADV_SWERVE_TIME = Range(0.5, 1.5)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for globalParameters.OPT_ADV_SWERVE_TIME seconds
    do LaneChangeBehavior(egoLaneSec, True, globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, is_oppositeTraffic=True)

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