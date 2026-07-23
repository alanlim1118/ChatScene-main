description = "Ego-vehicle encounters a pedestrian emerging from behind a parked vehicle and advancing into the lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(1, 3)

behavior AdvanceBehavior(speed):
    do WalkForwardBehavior(speed)

AdvAgent = new Pedestrian right of (ahead of egoSpawnPt by Range(10, 20)) by 3,
    facing 90 deg relative to egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvanceBehavior(globalParameters.OPT_ADV_SPEED)

parkedVehicle = new Car right of (ahead of egoSpawnPt by Range(25, 35)) by Range(3, 5),
    with regionContainedIn None