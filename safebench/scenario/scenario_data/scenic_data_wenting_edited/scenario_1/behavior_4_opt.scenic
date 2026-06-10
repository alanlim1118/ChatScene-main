description = "Ego vehicle encounters a pedestrian suddenly appearing from behind a parked car and stopping."
Town = globalParameters.town
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoTrajectory = PolylineRegion(globalParameters.waypoints)
param map = localPath(f'../maps/{Town}.xodr')
param carla_map = Town
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = globalParameters.weather

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 35)
param OPT_GEO_BLOCKER_X_OFFSET = Range(3, 5)
param OPT_PEDESTRIAN_BEHIND_DISTANCE = Range(2, 4)

blockerAheadPt = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
parkedCarSpawnPt = new OrientedPoint right of blockerAheadPt by globalParameters.OPT_GEO_BLOCKER_X_OFFSET
pedestrianSpawnPt = new OrientedPoint behind parkedCarSpawnPt by globalParameters.OPT_PEDESTRIAN_BEHIND_DISTANCE

param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_DIST = Range(5, 10)
param OPT_ADV_SPEED = Range(1.5, 2.5)
param OPT_ADV_DISTANCE = Range(15, 25)
param OPT_STOP_DISTANCE = Range(3, 5)

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_distance):
    start_pos = self.position
    do CrossingBehavior(actor_reference, min_speed=adv_speed, threshold=adv_distance) until (distance from self to start_pos >= stop_distance)
    take SetWalkingSpeedAction(0)
    while True:
        wait

ego = new Car at EgoSpawnPt,
    facing yaw,
    with regionContainedIn None,
    with blueprint MODEL

pedestrian = new Pedestrian at pedestrianSpawnPt,
    with heading pedestrianSpawnPt.heading + 90 deg,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, globalParameters.OPT_STOP_DISTANCE)

parkedCar = new Car at parkedCarSpawnPt,
    with blueprint MODEL

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightMonitor()
terminate when (distance from ego to EgoSpawnPt) > (globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE + 20)
terminate after 30 seconds
