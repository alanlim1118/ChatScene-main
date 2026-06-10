description = "Ego vehicle encounters an obstacle, performing emergency braking or avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
propSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(30, 50)

param EGO_SPEED = Range(10, 15)
param EGO_BRAKE = 1.0
SAFE_DIST = 20

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

prop = new Debris at propSpawnPt,
    with regionContainedIn None

require 30 <= (distance from egoSpawnPt to propSpawnPt) <= 50
terminate when (distance to egoSpawnPt) > 70
terminate after 30 seconds