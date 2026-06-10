description = "Target vehicle aggressively swerves into ego's lane, ego must avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

lane = Uniform(*filter(lambda l: any(s._laneToLeft or s._laneToRight for s in l.sections), network.lanes))
egoSection = Uniform(*filter(lambda s: s._laneToLeft or s._laneToRight, lane.sections))
leftSec = egoSection._laneToLeft
if leftSec is not None:
    advSection = leftSec
else:
    advSection = egoSection._laneToRight
advInitLane = advSection.lane
egoSpawnPt = new OrientedPoint in egoSection.centerline
advSpawnPt = new OrientedPoint in advSection.centerline

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_DISTANCE = Range(10, 15)

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

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