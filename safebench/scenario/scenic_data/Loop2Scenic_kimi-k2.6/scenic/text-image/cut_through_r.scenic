description = "The ego vehicle is going straight through the intersection; the adversarial vehicle approaches from the left front and cuts off the ego vehicle."

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

param OPT_EGO_SPEED = Range (1, 5)
param OPT_BRAKE_DISTANCE = Range(5, 8)

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
	interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
		take SetThrottleAction(0)  # Ensure no acceleration during braking
		take SetBrakeAction(1)  # Brake to avoid collision

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL,
	with behavior EgoBehavior()

param OPT_ADV_DISTANCE = Range(60, 70)  # Proximity within wich the adverserial car begins to drive
param OPT_EGO_SPEED = Range (1, 5)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1,1.2,1.3)

behavior WaitBehavior():
	while True:
		wait

behavior AdvBehavior():
	do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
	do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED,advTrajectory)
	terminate

AdvAgent = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with regionContainedIn None,
	with behavior AdvBehavior()

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

monitor TrafficLights():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 100):
			setClosestTrafficLightStatus(ego, "green")
		if withinDistanceToTrafficLight(AdvAgent, 100):
			setClosestTrafficLightStatus(AdvAgent, "red")
		wait

require monitor TrafficLights()
require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 10 <= (distance from advSpawnPt to intersection) <= 20
require 30 <= (distance from egoSpawnPt to intersection) <= 40

MODEL = 'vehicle.tesla.model3'

param weather = 'ClearNoon'
