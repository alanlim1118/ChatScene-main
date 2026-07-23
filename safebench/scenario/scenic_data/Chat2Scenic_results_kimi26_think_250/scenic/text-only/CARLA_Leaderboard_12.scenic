description = "Ego vehicle performs a lane change into moving traffic to avoid an obstacle blocking the lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 30)

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None:
            if laneSec._laneToRight.isForward == laneSec.isForward:
                laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
adjLaneSec = egoLaneSec._laneToRight
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
blockerSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
advSpawnPt = new OrientedPoint in adjLaneSec.centerline

param OPT_EGO_SPEED = Range(8, 12)
param OPT_EGO_AVOID_DIST = Range(10, 15)

behavior EgoBehavior(ego_speed, avoid_dist, lane_change_target):
    do FollowLaneBehavior(target_speed=ego_speed) until withinDistanceToObjsInLane(self, avoid_dist)
    do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=ego_speed)
    do FollowLaneBehavior(target_speed=ego_speed)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_AVOID_DIST,
        adjLaneSec
    )

adversary = new Car at advSpawnPt,
    with blueprint MODEL

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require 20 <= (distance from egoSpawnPt to blockerSpawnPt) <= 30
require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 35
terminate when (ego in adjLaneSec)