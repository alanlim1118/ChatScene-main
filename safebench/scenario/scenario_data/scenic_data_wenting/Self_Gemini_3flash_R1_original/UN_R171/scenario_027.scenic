description = "Ego vehicle travels straight, encounters stationary pedestrian in path, requiring autonomous braking to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE = Range(25, 35)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
pedestrianSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_THRESHOLD = Range(15, 20)

behavior EgoBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

behavior StationaryBehavior():
    while True:
        wait

pedestrian = new Pedestrian at pedestrianSpawnPt,
    with regionContainedIn None,
    with behavior StationaryBehavior()

TERM_DIST = 70

require 25 <= (distance from egoSpawnPt to pedestrianSpawnPt) <= 35
terminate when (distance to egoSpawnPt) > TERM_DIST