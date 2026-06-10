description = "Adversary vehicle suddenly exits ego vehicle's lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda l: l.orientation is not None and all([s._laneToLeft is not None or s._laneToRight is not None for s in l.sections]), network.lanes))
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(10, 20)

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = 1.0
SAFE_DIST = 10

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToObjsInLane(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

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
	with behavior AdversaryBehavior(globalParameters.ADV_SPEED)

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