description = "Ego vehicle reacts to a sudden lane change by a lead vehicle revealing a stationary obstacle, requiring collision avoidance."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_EGO_TO_LEAD = Range(10, 15)
param OPT_DIST_LEAD_TO_STATIONARY = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_EGO_TO_LEAD

stationarySpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_DIST_LEAD_TO_STATIONARY

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 10)
param OPT_ADV_TRIGGER_DIST = Range(10, 12)

behavior LeadCarBehavior(speed, trigger_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, trigger_dist)
    if self.laneSection._laneToLeft:
        target_lane_sec = self.laneSection._laneToLeft
    else:
        target_lane_sec = self.laneSection._laneToRight
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, is_oppositeTraffic=False, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

leadCar = new Car at leadSpawnPt,
    with behavior LeadCarBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_TRIGGER_DIST)

stationaryCar = new Car at stationarySpawnPt

require 10 <= (distance from egoSpawnPt to leadSpawnPt) <= 15
require 15 <= (distance from leadSpawnPt to stationarySpawnPt) <= 25

terminate when (distance to egoSpawnPt) > 80
