description = "Ego vehicle encounters a stopped vehicle and performs emergency braking or avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param DISTANCE_TO_STATIONARY = Range(20, 40)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
stationaryCarSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.DISTANCE_TO_STATIONARY

param EGO_SPEED = Range(10, 15)
param EGO_BRAKE = 1.0
param SAFETY_DISTANCE = Range(15, 20)

behavior EgoBehavior(speed, safety_dist, brake_val):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE, globalParameters.EGO_BRAKE)

behavior AdversaryBehavior():
    while True:
        wait

adversary = new Car at stationaryCarSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require 20 <= (distance from egoSpawnPt to stationaryCarSpawnPt) <= 40
terminate when (distance from ego to adversary) > 50
terminate after 30 seconds