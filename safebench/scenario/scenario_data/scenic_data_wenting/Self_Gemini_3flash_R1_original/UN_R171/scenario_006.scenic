description = "Ego vehicle performs a lane change into a conflicting space with another merging vehicle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

targetLane = Uniform(*filter(lambda l: all(s._laneToLeft is not None and s._laneToRight is not None for s in l.sections), network.lanes))
targetSection = Uniform(*targetLane.sections)
egoSection = targetSection._laneToRight
advSection = targetSection._laneToLeft

egoSpawnPt = new OrientedPoint in egoSection.centerline
advSpawnPt = new OrientedPoint in advSection.centerline

require 5 < (distance from egoSpawnPt to advSpawnPt) < 15

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = 10

behavior EgoBehavior(target_speed, target_lane, safety_dist):
    try:
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, safety_dist):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, targetSection, globalParameters.SAFETY_DISTANCE)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(target_speed, target_lane):
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED, targetSection)

require 5 < (distance from egoSpawnPt to advSpawnPt) < 15
terminate when (distance from ego to egoSpawnPt) > 100
terminate after 40 seconds