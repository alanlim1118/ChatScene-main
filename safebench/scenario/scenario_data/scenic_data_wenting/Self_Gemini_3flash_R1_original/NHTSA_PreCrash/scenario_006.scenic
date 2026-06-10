description = "Ego vehicle travels straight in a rural area at night, under a high speed limit, and departs the road at a non-junction."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
egoTrajectory = [egoInitLane]

param EGO_SPEED = Range(15, 20)
param PRE_DEPART_DISTANCE = Range(30, 50)

behavior EgoBehavior(speed, drive_dist):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to egoSpawnPt) > drive_dist
    while True:
        take SetSteerAction(0.5), SetThrottleAction(0.5)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.PRE_DEPART_DISTANCE)

require (distance from egoSpawnPt to intersection) > 50

terminate when (distance from ego to egoSpawnPt) > (globalParameters.PRE_DEPART_DISTANCE + 10) and ego not in network.roads
terminate after 20 seconds