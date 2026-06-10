description = "Ego vehicle maintains lane stability with an adjacent close vehicle and a swerving leading vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(15, 25)
param OPT_ADJ_OFFSET = Range(-5, 5)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

egoLaneSec = network.laneSectionAt(egoSpawnPt)

if egoLaneSec._laneToLeft is not None and egoLaneSec._laneToLeft.isForward:
    adjLaneSec = egoLaneSec._laneToLeft
elif egoLaneSec._laneToRight is not None and egoLaneSec._laneToRight.isForward:
    adjLaneSec = egoLaneSec._laneToRight

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjBasePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjBasePt for globalParameters.OPT_ADJ_OFFSET

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param SWERVE_STEER = 0.15

behavior SwervingBehavior(speed):
    take SetSpeedAction(speed)
    while True:
        take SetSteerAction(globalParameters.SWERVE_STEER)
        wait for 0.4 seconds
        take SetSteerAction(-globalParameters.SWERVE_STEER)
        wait for 0.4 seconds

adversary_leading = new Car at LeadingSpawnPt,
    with blueprint MODEL,
    with behavior SwervingBehavior(globalParameters.OPT_ADV_SPEED)

require 15 <= (distance from egoSpawnPt to LeadingSpawnPt) <= 25
require (distance from egoSpawnPt to AdvSpawnPt) <= 10
terminate after 30 seconds
terminate when (distance from ego to adversary_leading) > 80
