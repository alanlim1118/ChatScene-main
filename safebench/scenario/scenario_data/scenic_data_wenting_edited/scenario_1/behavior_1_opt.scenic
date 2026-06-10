description = "Ego vehicle encounters a pedestrian suddenly crossing from the right front and stopping."
Town = globalParameters.town
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoTrajectory = PolylineRegion(globalParameters.waypoints)
param map = localPath(f'../maps/{Town}.xodr')
param carla_map = Town
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = globalParameters.weather

param OPT_DISTANCE_AHEAD = Range(20, 30)
param OPT_DISTANCE_RIGHT = Range(4, 8)

IntSpawnPt = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OPT_DISTANCE_AHEAD
pedSpawnPt = new OrientedPoint right of IntSpawnPt by globalParameters.OPT_DISTANCE_RIGHT, facing (90 deg) relative to IntSpawnPt.heading

param OPT_PED_SPEED = Range(1.0, 2.0)
param OPT_PED_THRESHOLD = Range(20, 25)
param OPT_STOP_THRESHOLD = 0.8

behavior PedestrianBehavior(target_actor, speed, threshold, stop_point, stop_dist):
    do CrossingBehavior(target_actor, min_speed=speed, threshold=threshold) until (distance from self to stop_point <= stop_dist)
    take SetWalkingSpeedAction(0)
    terminate

ego = new Car at EgoSpawnPt,
    facing yaw,
    with regionContainedIn None,
    with blueprint MODEL

ped = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianBehavior(ego, globalParameters.OPT_PED_SPEED, globalParameters.OPT_PED_THRESHOLD, IntSpawnPt, globalParameters.OPT_STOP_THRESHOLD)

TERM_DIST = globalParameters.OPT_DISTANCE_AHEAD + 10
terminate when (distance from ego to EgoSpawnPt) > TERM_DIST
