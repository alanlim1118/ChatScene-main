description = "Ego-vehicle encounters a pedestrian or bicycle and must perform an emergency brake or an avoidance maneuver."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

lane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in lane.centerline
advSpawnPt = new OrientedPoint following lane.orientation from egoSpawnPt for Range(20, 40)

param OPT_EGO_SPEED = Range(7, 10)
param OPT_EGO_BRAKE = Range(0.5, 1.0)
param OPT_OBSTACLE_DIST = Range(8, 12)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_OBSTACLE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.OPT_EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Pedestrian at advSpawnPt