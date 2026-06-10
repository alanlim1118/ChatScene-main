description = "Vehicle A attempts an unsafe pass of Vehicle B, resulting in a collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLaneSec = network.laneSectionAt(egoSpawnPt)

param OPT_DISTANCE = Range(10, 15)

advSpawnPt = new OrientedPoint following egoInitLaneSec.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 12)

behavior AdversaryBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED)

EGO_ADV_DIST = [10, 15]

require egoInitLaneSec._laneToLeft is not None and egoInitLaneSec._laneToLeft.isForward
require EGO_ADV_DIST[0] <= (distance from egoSpawnPt to advSpawnPt) <= EGO_ADV_DIST[1]
terminate when ego intersects adv
