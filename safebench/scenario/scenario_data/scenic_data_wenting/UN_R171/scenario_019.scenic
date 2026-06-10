description = "Lead vehicle performs an emergency lane change to avoid a stopped car at low TTC, creating a high-urgency late reveal for ego."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_TO_LEAD_DIST = Range(10, 15)
param OPT_LEAD_TO_STOPPED_DIST = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_TO_LEAD_DIST
stoppedSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_LEAD_TO_STOPPED_DIST

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_LEAD_SPEED = Range(12, 15)
param LEAD_THRESHOLD = 10

behavior LeadBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, threshold)
    if self.laneSection._laneToLeft:
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=speed)
    elif self.laneSection._laneToRight:
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

stopped = new Car at stoppedSpawnPt,
    with blueprint MODEL

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(globalParameters.OPT_LEAD_SPEED, globalParameters.LEAD_THRESHOLD)


require 15 <= (distance from leadSpawnPt to stoppedSpawnPt) <= 25
terminate when (distance from ego to egoSpawnPt) > 80
