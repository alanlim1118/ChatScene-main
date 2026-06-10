description = "Ego vehicle reacts to a sudden lane change by a lead vehicle revealing a stationary obstacle, requiring collision avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param DIST_EGO_TO_LEAD = Range(10, 15)
param DIST_LEAD_TO_STATIONARY = Range(15, 25)

laneSecsWithAdjacent = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward:
            valid_left = sec._laneToLeft is not None and sec._laneToLeft.isForward
            valid_right = sec._laneToRight is not None and sec._laneToRight.isForward
            if valid_left or valid_right:
                laneSecsWithAdjacent.append(sec)

egoLaneSec = Uniform(*laneSecsWithAdjacent)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline


leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

stationarySpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.DIST_LEAD_TO_STATIONARY

param OPT_EGO_SPEED = Range(8, 10)
param OPT_BRAKE_THRESHOLD = Range(7, 9)

behavior EgoBehavior(speed, brake_dist):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

param ADV_SPEED = Range(8, 10)
param ADV_TRIGGER_DIST = Range(10, 12)

behavior LeadCarBehavior(speed, trigger_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, trigger_dist)
    if self.laneSection._laneToLeft:
        target_lane_sec = self.laneSection._laneToLeft
    else:
        target_lane_sec = self.laneSection._laneToRight
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, is_oppositeTraffic=False, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

leadCar = new Car at leadSpawnPt,
    with behavior LeadCarBehavior(globalParameters.ADV_SPEED, globalParameters.ADV_TRIGGER_DIST)

stationaryCar = new Car at stationarySpawnPt

require 10 <= (distance from egoSpawnPt to leadSpawnPt) <= 15
require 15 <= (distance from leadSpawnPt to stationarySpawnPt) <= 25

terminate when (distance to egoSpawnPt) > 80