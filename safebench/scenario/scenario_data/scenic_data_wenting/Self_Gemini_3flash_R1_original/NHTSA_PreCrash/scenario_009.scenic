description = "Ego vehicle drives straight in an urban area and encounters a pedestrian at a non-junction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param pedDist = Range(15, 25)
param pedOffset = Range(3, 5)
egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoLane.centerline

tempPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.pedDist
pedSpawnPt = new OrientedPoint right of tempPt by globalParameters.pedOffset

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing -90 deg relative to ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

require 15 <= (distance from egoSpawnPt to pedSpawnPt) <= 30
terminate when (distance from ego to egoSpawnPt) > (globalParameters.pedDist + 20)