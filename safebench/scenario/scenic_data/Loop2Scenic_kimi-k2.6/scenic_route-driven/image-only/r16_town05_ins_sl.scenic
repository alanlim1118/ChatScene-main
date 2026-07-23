description = "The ego vehicle is approaching the intersection; the adversarial car (on the left) suddenly accelerates and enters the intersection first and suddenly stop."

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.intersection is not None and m.intersection.is4Way and m.intersection.isSignalized, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advTrajectoryLine = advManeuver.startLane.centerline + advManeuver.connectingLane.rightEdge + advManeuver.endLane.centerline
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

from scenic.domains.driving.controllers import *

param OPT_ADV_THROTTLE = Range(5, 10)/10
param OPT_ADV_TIMER = Range(10, 30)/10
param OPT_ADV_SPEED = Range (1, 5) * Range(8,12)/10
OPT_ADV_DISTANCE = 1

behavior AccelerateBehavior(throttle, trajectory):
	"""
	Accelerates the vehicle with a fixed throttle value while maintaining lateral stability
	using a PIDLateralController. The vehicle follows the given trajectory and adjusts its
	steering angle to minimize cross-track error.
	"""

	lateral_controller = PIDLateralController(K_P=0.3, K_D=0.2, K_I=0, dt=0.1)

	past_steer_angle = 0  # Initialize past steering angle

	while True:
		cte = trajectory.signedDistanceTo(self.position)  # Distance to the trajectory centerline

		steering_angle = lateral_controller.run_step(cte)

		take RegulatedControlAction(throttle, steering_angle, past_steer_angle)

		past_steer_angle = steering_angle

behavior AdvBehavior():
	do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to intersection <= OPT_ADV_DISTANCE)
	do AccelerateBehavior(throttle=globalParameters.OPT_ADV_THROTTLE, trajectory=advTrajectoryLine) for globalParameters.OPT_ADV_TIMER seconds
	take SetThrottleAction(0)
	take SetBrakeAction(1)

AdvAgent = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with regionContainedIn None,
	with behavior AdvBehavior()

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

require CONST_MIN_LEFT_DEG < (egoDir - advDir) < CONST_MAX_LEFT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
