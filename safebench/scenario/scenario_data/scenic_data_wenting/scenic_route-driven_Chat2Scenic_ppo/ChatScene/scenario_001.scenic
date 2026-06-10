description = "Ego vehicle encounters a pedestrian suddenly crossing from the right front and stopping."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE_AHEAD = Range(20, 30)
param OPT_DISTANCE_RIGHT = Range(4, 8)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

IntSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DISTANCE_AHEAD
pedSpawnPt = new OrientedPoint right of IntSpawnPt by globalParameters.OPT_DISTANCE_RIGHT, facing (90 deg) relative to IntSpawnPt.heading

param OPT_EGO_SPEED = Range(7, 10)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_PED_SPEED = Range(1.0, 2.0)
param OPT_PED_THRESHOLD = Range(20, 25)
param OPT_STOP_THRESHOLD = 0.8

behavior PedestrianBehavior(target_actor, speed, threshold, stop_point, stop_dist):
    do CrossingBehavior(target_actor, min_speed=speed, threshold=threshold) until (distance from self to stop_point <= stop_dist)
    take SetWalkingSpeedAction(0)
    terminate

ped = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianBehavior(ego, globalParameters.OPT_PED_SPEED, globalParameters.OPT_PED_THRESHOLD, IntSpawnPt, globalParameters.OPT_STOP_THRESHOLD)

TERM_DIST = globalParameters.OPT_DISTANCE_AHEAD + 10
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
