description = "Ego vehicle encounters a pedestrian suddenly appearing from a driveway, stopping, and walking diagonally."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PED_AHEAD = Range(20, 35)
param OPT_DRIVEWAY_OFFSET = Range(6, 10)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

targetPointOnRoad = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_PED_AHEAD
pedSpawnPt = new OrientedPoint left of targetPointOnRoad by globalParameters.OPT_DRIVEWAY_OFFSET, facing toward targetPointOnRoad

egoTrajectory = [egoInitLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

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
require 20 <= globalParameters.OPT_PED_AHEAD <= 35
require 6 <= globalParameters.OPT_DRIVEWAY_OFFSET <= 10
terminate when (distance from ego to egoSpawnPt) > 80