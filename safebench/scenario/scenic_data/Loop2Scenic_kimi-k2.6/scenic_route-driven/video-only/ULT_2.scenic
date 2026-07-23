description = "The ego is driving straight through an intersection when a crossing vehicle runs the red light and unexpectedly accelerates, forcing the ego to quickly reassess the situation and perform a collision avoidance maneuver."

Town = 'Town05'
param map = localPath('../../maps/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way and m.intersection.isSignalized, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

param OPT_ADV_SPEED = Range(1, 5)
param OPT_ADV_DISTANCE = Range(30, 50)
param OPT_ADV_ACC_DIST = Range(13,20)
param OPT_ADV_THROTTLE = Range(5, 10) / 10

behavior WaitBehavior():
	while True:
		wait

behavior AdvBehavior():
	do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego) < globalParameters.OPT_ADV_ACC_DIST
	take SetThrottleAction(globalParameters.OPT_ADV_THROTTLE)

AdvAgent = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with regionContainedIn None,
	with behavior AdvBehavior()

monitor TrafficLights():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 100):
			setClosestTrafficLightStatus(ego, "green")
		if withinDistanceToTrafficLight(AdvAgent, 100):
			setClosestTrafficLightStatus(AdvAgent, "red")
		wait

require monitor TrafficLights()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 5 <= (distance from advSpawnPt to intersection) <= 10

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
