description = "Ego vehicle maintains lane stability with an adjacent close vehicle and a swerving leading vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(15, 25)
param OPT_ADJ_OFFSET = Range(-5, 5)

lanePairs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward:
            if sec._laneToLeft is not None and sec._laneToLeft.isForward:
                lanePairs.append((sec, sec._laneToLeft))
            elif sec._laneToRight is not None and sec._laneToRight.isForward:
                lanePairs.append((sec, sec._laneToRight))

selected = Uniform(*lanePairs)
egoLaneSec = selected[0]
adjLaneSec = selected[1]

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjBasePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjBasePt for globalParameters.OPT_ADJ_OFFSET

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(10, 15)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param ADV_SPEED = Range(10, 15)
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
    with behavior SwervingBehavior(globalParameters.ADV_SPEED)

require 15 <= (distance from egoSpawnPt to LeadingSpawnPt) <= 25
require (distance from egoSpawnPt to AdvSpawnPt) <= 10
terminate after 30 seconds
terminate when (distance from ego to adversary_leading) > 80