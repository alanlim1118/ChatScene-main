description = "Ego vehicle encounters an obstacle, performing emergency braking or avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

param OPT_DISTANCE = Range(30, 50)

propSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

prop = new Debris at propSpawnPt,
    with regionContainedIn None

require 30 <= (distance from egoSpawnPt to propSpawnPt) <= 50
terminate when (distance to egoSpawnPt) > 70
terminate after 30 seconds
