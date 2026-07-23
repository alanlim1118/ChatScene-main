description = "Green vehicle approaches a pedestrian on a two-way road near a stationary blue car with driving beams activated."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(8, 12)
param APPROACH_DIST = 15.0

behavior EgoBehavior(speed, approach_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToAnyPedestrians(self, approach_dist)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(EGO_SPEED, APPROACH_DIST)

param PED_MIN_SPEED = Range(1.0, 1.5)
param PED_THRESHOLD = 15.0

behavior PedCrossingBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

adversarial = new Pedestrian right of egoSpawnPt by 5,
    facing 90 deg relative to egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedCrossingBehavior()

stationary = new Car ahead of egoSpawnPt by Range(10, 20),
    with blueprint MODEL

require 10 <= (distance from egoSpawnPt to stationary) <= 20