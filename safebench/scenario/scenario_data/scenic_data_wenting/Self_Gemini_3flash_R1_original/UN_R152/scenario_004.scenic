description = "Subject vehicle performs a lane change to avoid a lane reduction signboard."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*filter(lambda l: any(s._laneToLeft or s._laneToRight for s in l.sections), network.lanes))
egoSection = Uniform(*filter(lambda s: s._laneToLeft or s._laneToRight, egoInitLane.sections))

egoSpawnPt = new OrientedPoint on egoSection.centerline
warningSign = new TrafficWarning following egoSection.orientation from egoSpawnPt for Range(20, 30)

param OPT_EGO_SPEED = Range(7, 10)
param OPT_LC_TRIGGER_DIST = Range(15, 20)
param OPT_BRAKE_DIST = 5

behavior EgoBehavior(speed, lc_dist, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed) until (distance from self to warningSign < lc_dist)
        target_sec = egoSection._laneToLeft if egoSection._laneToLeft else egoSection._laneToRight
        do LaneChangeBehavior(laneSectionToSwitch=target_sec, target_speed=speed)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetBrakeAction(1)
        terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_LC_TRIGGER_DIST, globalParameters.OPT_BRAKE_DIST)

param ADV_SPEED = Range(7, 10)
param ADV_DIST = Range(10, 15)

behavior AdversaryBehavior(speed):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to warningSign < 15)
    do LaneChangeBehavior(laneSectionToSwitch=(egoSection._laneToLeft if egoSection._laneToLeft else egoSection._laneToRight), target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car following egoSection.orientation from egoSpawnPt for globalParameters.ADV_DIST,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED)

require 10 <= (distance from ego to adversary) <= 15
require 20 <= (distance from ego to warningSign) <= 30
terminate when (distance from ego to egoSpawnPt) > 60
terminate after 500 steps