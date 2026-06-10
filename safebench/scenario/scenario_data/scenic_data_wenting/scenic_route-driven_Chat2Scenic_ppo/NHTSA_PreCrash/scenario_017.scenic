description = "Vehicle drifts and encroaches into an oncoming vehicle while going straight in a rural non-junction area with a high speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

advSection = egoSection._laneToLeft
require advSection is not None
require advSection.isForward != egoSection.isForward

parallelVec = advSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advSection.orientation from parallelVec for Range(-100, -60)

advTrajectory = [advSection.lane]

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

require 60 <= (distance from egoSpawnPt to advSpawnPt) <= 100

terminate when (distance from ego to egoSpawnPt) > 150
terminate after 15 seconds
