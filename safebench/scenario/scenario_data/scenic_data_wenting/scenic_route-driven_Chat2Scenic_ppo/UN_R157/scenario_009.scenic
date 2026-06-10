description = "Ego vehicle performs emergency braking after a lead vehicle reveals a stationary obstacle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DISTANCE = Range(10, 15)
param OPT_STATIONARY_DISTANCE = Range(20, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DISTANCE
stationarySpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_STATIONARY_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)
param OPT_REVEAL_DIST = Range(10, 15)

behavior AdversaryBehavior(speed, reveal_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, reveal_dist)
    if self.laneSection.laneToLeft:
        dest_lane = self.laneSection.laneToLeft
        do LaneChangeBehavior(laneSectionToSwitch=dest_lane, target_speed=speed)
    elif self.laneSection.laneToRight:
        dest_lane = self.laneSection.laneToRight
        do LaneChangeBehavior(laneSectionToSwitch=dest_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_REVEAL_DIST)

stationaryObstacle = new Car at stationarySpawnPt,
    with heading stationarySpawnPt.heading,
    with regionContainedIn None

require 30 <= (distance from egoSpawnPt to stationarySpawnPt) <= 45

terminate when (distance from ego to stationarySpawnPt) > 50
terminate when ego.speed < 0.1 and (distance from ego to egoSpawnPt) > 5
