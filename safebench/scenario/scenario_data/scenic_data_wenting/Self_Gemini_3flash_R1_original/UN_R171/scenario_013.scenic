description = "Ego vehicle approaches a slower target vehicle directly ahead in the same lane with a constant speed difference."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_AHEAD = Range(15, 25)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline
npcSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

param OPT_EGO_SPEED = 15

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

param OPT_NPC_SPEED = globalParameters.OPT_EGO_SPEED - 5

npc = new NPCCar at npcSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

require 15 <= (distance from egoSpawnPt to npcSpawnPt) <= 25