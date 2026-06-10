description = "Ego vehicle encounters a pedestrian suddenly appearing from behind a parked car and stopping."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 35)
param OPT_GEO_BLOCKER_X_OFFSET = Range(3, 5)
param OPT_PEDESTRIAN_BEHIND_DISTANCE = Range(2, 4)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

blockerAheadPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
parkedCarSpawnPt = new OrientedPoint right of blockerAheadPt by globalParameters.OPT_GEO_BLOCKER_X_OFFSET

pedestrianSpawnPt = new OrientedPoint behind parkedCarSpawnPt by globalParameters.OPT_PEDESTRIAN_BEHIND_DISTANCE

egoTrajectory = [egoInitLane]

param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_DIST = Range(5, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(1.5, 2.5)
param OPT_ADV_DISTANCE = Range(15, 25)
param OPT_STOP_DISTANCE = Range(3, 5)

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_distance):
    start_pos = self.position
    do CrossingBehavior(actor_reference, min_speed=adv_speed, threshold=adv_distance) until (distance from self to start_pos >= stop_distance)
    take SetWalkingSpeedAction(0)
    while True:
        wait

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
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE + 20)
terminate after 30 seconds