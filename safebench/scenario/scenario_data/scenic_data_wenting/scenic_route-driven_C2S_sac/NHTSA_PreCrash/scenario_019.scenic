description = "Ego vehicle approaches an accelerating lead vehicle at an urban intersection with a 45 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_BEHIND_ADV = Range(10, 15)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_EGO_BEHIND_ADV
advTrajectory = [l for l in [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane] if l is not None]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_INIT_SPEED = Range(5, 7)
param OPT_ADV_END_SPEED = Range(12, 15)
param OPT_ADV_TRIGGER_DIST = Range(10, 12)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_INIT_SPEED, trajectory=trajectory) \
		until (distance from self to ego) < globalParameters.OPT_ADV_TRIGGER_DIST
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_END_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

TERM_DIST = 70

monitor TrafficLights():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 50):
			setClosestTrafficLightStatus(ego, "green")
		if withinDistanceToTrafficLight(adversary, 50):
			setClosestTrafficLightStatus(adversary, "green")
		wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 15
terminate when (distance to egoSpawnPt) > TERM_DIST
