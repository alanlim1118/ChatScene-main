description = "Ego vehicle travels on a rural road when a bicyclist on the shoulder suddenly turns left across its path, causing an emergency braking and swerving scenario that results in a side collision."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

encounterPt = new OrientedPoint ahead of egoSpawnPt by 30
advSpawnPt = new OrientedPoint right of encounterPt by 3.0, facing encounterPt.heading

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_BICYCLE_FWD_TIME = Range(1, 3)
param BICYCLE_TURN_STEER = -0.9
param BICYCLE_THROTTLE = 0.5

behavior BicycleAdvBehavior():
    do AccelerateForwardBehavior() for globalParameters.OPT_BICYCLE_FWD_TIME seconds
    while True:
        take SetSteerAction(globalParameters.BICYCLE_TURN_STEER)
        take SetThrottleAction(globalParameters.BICYCLE_THROTTLE)

bicycle = new Bicycle at advSpawnPt,
    facing advSpawnPt.heading,
    with behavior BicycleAdvBehavior()

TERM_DIST = 20
MIN_TRAVEL = 45

terminate when (distance from ego to bicycle) > TERM_DIST and (distance to egoSpawnPt) > MIN_TRAVEL