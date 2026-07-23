description = "Ego vehicle travels straight through a T-junction where a pedestrian crosses from the left and a grey vehicle turns left from the side road to merge behind."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian left of egoSpawnPt by 4,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

param EGO_TRAVEL_DISTANCE = 60

require 20 <= (distance from ego to intersection) <= 50
require 3 <= (distance from ego to ped) <= 10
require 15 <= (distance from ego to merger) <= 40

terminate when (distance from egoSpawnPt to ego) >= globalParameters.EGO_TRAVEL_DISTANCE