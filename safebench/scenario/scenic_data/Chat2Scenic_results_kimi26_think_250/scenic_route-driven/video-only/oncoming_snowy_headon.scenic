description = "Ego vehicle loses control on snow and collides head-on with an oncoming sedan after a taxi cuts across its lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_TAXI_SEDAN_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
oncomingLaneSec = egoLaneSec._laneToLeft
taxiSpawnPt = new OrientedPoint in oncomingLaneSec.centerline
sedanSpawnPt = new OrientedPoint behind taxiSpawnPt by globalParameters.OPT_TAXI_SEDAN_DIST

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_SEDAN_SPEED = Range(8, 12)

behavior SedanBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

sedan = new Car at sedanSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with behavior SedanBehavior(globalParameters.OPT_SEDAN_SPEED)

require (distance from ego to sedan) > 5
terminate when (distance from ego to sedan) < 4