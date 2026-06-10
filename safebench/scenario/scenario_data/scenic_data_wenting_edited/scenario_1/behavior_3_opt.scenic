description = "Ego vehicle encounters a pedestrian suddenly appearing from a driveway, stopping, and walking diagonally."
Town = globalParameters.town
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoTrajectory = PolylineRegion(globalParameters.waypoints)
param map = localPath(f'../maps/{Town}.xodr')
param carla_map = Town
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = globalParameters.weather

param OPT_PED_AHEAD = Range(20, 35)
param OPT_DRIVEWAY_OFFSET = Range(6, 10)

targetPointOnRoad = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OPT_PED_AHEAD
pedSpawnPt = new OrientedPoint left of targetPointOnRoad by globalParameters.OPT_DRIVEWAY_OFFSET, facing toward targetPointOnRoad

param OPT_ADV_SPEED = Range(1.2, 1.8)
param OPT_WAIT_SEC_STOP = Range(1.0, 3.0)
param OPT_DIAGONAL_ANGLE = Range(-45, 45)

behavior WaitBehavior():
    while True:
        wait

behavior AdversarialBehavior(speed, stop_time, diagonal_angle):
    take SetWalkingSpeedAction(0)
    do WaitBehavior() for stop_time seconds
    take SetWalkingDirectionAction(self.heading + diagonal_angle deg)
    do WalkForwardBehavior(speed)

ego = new Car at EgoSpawnPt,
    facing yaw,
    with regionContainedIn None,
    with blueprint MODEL

advPedestrian = new Pedestrian at pedSpawnPt,
    with behavior AdversarialBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_WAIT_SEC_STOP, globalParameters.OPT_DIAGONAL_ANGLE),
    with regionContainedIn None

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 20 <= OPT_PED_AHEAD <= 35
require 6 <= OPT_DRIVEWAY_OFFSET <= 10
terminate when (distance from ego to EgoSpawnPt) > 80
