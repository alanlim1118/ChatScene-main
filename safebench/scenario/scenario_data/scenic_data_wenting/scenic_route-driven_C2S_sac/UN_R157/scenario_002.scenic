description = "Adversary vehicle suddenly exits ego vehicle's lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(10, 20)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed) for Range(2, 4) seconds
    if self.laneSection._laneToLeft is not None:
        target_lane_sec = self.laneSection._laneToLeft
    else:
        target_lane_sec = self.laneSection._laneToRight
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 20
terminate when (distance from ego to adversary) > 50
terminate after 20 seconds
