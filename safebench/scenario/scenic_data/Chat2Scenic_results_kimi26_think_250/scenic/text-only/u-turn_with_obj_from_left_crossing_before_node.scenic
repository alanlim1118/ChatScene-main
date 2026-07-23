description = "Ego vehicle executes a U-turn at an intersection while an adversarial object crosses from the left before the intersection node."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(3, 6)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(1, 2)

AdvAgent = new Pedestrian left of (ahead of egoSpawnPt by Range(8, 12)) by Range(3, 5),
    facing -90 deg relative to egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior WalkForwardBehavior(speed=globalParameters.OPT_ADV_SPEED)