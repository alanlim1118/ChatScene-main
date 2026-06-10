description = "Ego vehicle loses control on wet roads in a rural area, running off the road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoDir = egoSpawnPt.heading
egoTrajectory = [egoInitLane]

param EGO_SPEED = Range(12, 15)
param STEER_AMOUNT = Range(0.5, 0.8)
param DRIVE_DURATION = Range(2, 4)

behavior EgoBehavior(trajectory, speed, steer, duration):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory) for duration seconds
    while True:
        take SetSteerAction(steer)
        take SetThrottleAction(0.5)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED, globalParameters.STEER_AMOUNT, globalParameters.DRIVE_DURATION)

require (distance from egoSpawnPt to intersection) > 50