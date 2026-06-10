description = "Ego vehicle approaches a slower target vehicle directly ahead in the same lane with a constant speed difference."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_AHEAD = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

npcSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_NPC_SPEED = 10

npc = new NPCCar at npcSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

require 15 <= (distance from egoSpawnPt to npcSpawnPt) <= 25
