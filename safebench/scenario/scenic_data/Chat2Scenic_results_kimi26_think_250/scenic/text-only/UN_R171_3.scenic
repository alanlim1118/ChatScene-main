description = "Ego vehicle delays lane change to yield to a high-speed overtaking vehicle in the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SAFE_DIST = Range(20, 30)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (withinDistanceToAnyCars(self, globalParameters.OPT_SAFE_DIST))
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (not withinDistanceToAnyCars(self, globalParameters.OPT_SAFE_DIST))
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED + Uniform(8, 12)

OvertakingCar = new Car behind ego by Range(40, 60),
    with regionContainedIn egoLaneSec._laneToLeft,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

terminate when (distance from OvertakingCar to ego) > 70