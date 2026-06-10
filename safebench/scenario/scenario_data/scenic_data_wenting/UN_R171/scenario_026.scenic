description = "Target vehicle aggressively swerves into ego's lane, ego must avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

advSection = egoSection._laneToLeft if egoSection._laneToLeft is not None else egoSection._laneToRight
require advSection is not None
advInitLane = advSection.lane
advSpawnPt = new OrientedPoint in advSection.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(12, 17)
param OPT_SWERVE_DISTANCE = Range(10, 15)

behavior AdversaryBehavior(speed, swerve_dist, target_sec):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < swerve_dist
    do LaneChangeBehavior(laneSectionToSwitch=target_sec, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_SWERVE_DISTANCE, egoSection)

require (not ego intersects adversary)
terminate when (distance from ego to adversary) > 70
