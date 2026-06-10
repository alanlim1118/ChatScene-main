description = "Ego vehicle encounters a stationary heavy truck in a curve and brakes to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

truckSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(50, 70)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

truck = new Truck at truckSpawnPt,
    with heading truckSpawnPt.heading,
    with regionContainedIn None

require 50 <= (distance from egoSpawnPt to truckSpawnPt) <= 70
terminate after 30 seconds
