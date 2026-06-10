description = "Ego vehicle follows a slow lead vehicle on a straight highway, then performs a lane change to overtake it."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)
egoInitLane = egoSection.lane

leadSpawnPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for Range(20, 30)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoSection,
    with blueprint MODEL

param OPT_NPC_SPEED = Range(5, 10)

behavior NPCBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

npcCar = new NPCCar at leadSpawnPt,
    with behavior NPCBehavior(globalParameters.OPT_NPC_SPEED)

require 20 <= (distance from egoSpawnPt to leadSpawnPt) <= 30
require ego can see npcCar
terminate after 30 seconds
