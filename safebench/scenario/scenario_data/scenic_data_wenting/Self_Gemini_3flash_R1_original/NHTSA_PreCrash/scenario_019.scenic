description = "Ego vehicle approaches an accelerating lead vehicle at an urban intersection with a 45 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST_TO_INT = Range(10, 15)
param OPT_EGO_BEHIND_ADV = Range(10, 15)

intersection = Uniform(*network.intersections)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane

advSpawnPt = new OrientedPoint following roadDirection from egoInitLane.centerline.end for -globalParameters.OPT_ADV_DIST_TO_INT
egoSpawnPt = new OrientedPoint behind advSpawnPt by globalParameters.OPT_EGO_BEHIND_ADV

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

param OPT_EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_INIT_SPEED = Range(5, 7)
param ADV_END_SPEED = Range(12, 15)
param ADV_TRIGGER_DIST = Range(10, 12)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_INIT_SPEED, trajectory=trajectory) \
		until (distance from self to ego) < globalParameters.ADV_TRIGGER_DIST
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_END_SPEED, trajectory=trajectory)

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