description = "Ego drives straight; pedestrian sprints from behind bus stop onto road and stops."
Town = globalParameters.town
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoTrajectory = PolylineRegion(globalParameters.waypoints)
param map = localPath(f'../maps/{Town}.xodr')
param carla_map = Town
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = globalParameters.weather

param OPT_GEO_BLOCKER_DISTANCE = Range(20, 35)
param OPT_GEO_PED_X_OFFSET = Range(-0.5, 0.5)
param OPT_GEO_PED_Y_OFFSET = Range(1.0, 2.5)

egoInitLane = network.laneAt(EgoSpawnPt)

busStopAnchorPt = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OPT_GEO_BLOCKER_DISTANCE
busStopSpawnPt = new OrientedPoint right of busStopAnchorPt by 4

PED_SHIFT = globalParameters.OPT_GEO_PED_X_OFFSET @ globalParameters.OPT_GEO_PED_Y_OFFSET
pedSpawnPt = new OrientedPoint at busStopSpawnPt offset by PED_SHIFT

param OPT_ADV_SPEED = Range(2.5, 4.5)
param OPT_ADV_THRESHOLD = Range(15, 25)
param OPT_STOP_DISTANCE = Range(0.5, 1.5)

behavior PedestrianBehavior(actor_reference, target_lane, adv_speed, threshold, stop_dist):
    do CrossingBehavior(actor_reference, min_speed=adv_speed, threshold=threshold) until (distance from self to target_lane.centerline <= stop_dist)
    take SetWalkingSpeedAction(0)
    while True:
        wait

ego = new Car at EgoSpawnPt,
    facing yaw,
    with regionContainedIn None,
    with blueprint MODEL

ped = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedSpawnPt.heading,
    with behavior PedestrianBehavior(ego, egoInitLane, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_THRESHOLD, globalParameters.OPT_STOP_DISTANCE)

busStop = new BusStop at busStopSpawnPt,
    with heading busStopSpawnPt.heading,
    with regionContainedIn None

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 40 <= (distance from EgoSpawnPt to intersection) <= 60
terminate when (distance from ego to EgoSpawnPt) > (globalParameters.OPT_GEO_BLOCKER_DISTANCE + 20)
