"""Scenario Description:

The ego car travels straight forward through the intersection. Simultaneously, an adversarial object crosses the street from the right side before reaching the intersection node.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_ADV_THROTTLE = Range(6, 9)/10
param OPT_ADV_TIMER = Range(8, 20)/10
param OPT_ADV_SPEED = Range(2, 5) * Range(9, 13)/10

OPT_ADV_DISTANCE = 1

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior AccelerateBehavior(throttle, trajectory):
    """
    Accelerates the vehicle with a fixed throttle value while maintaining lateral stability
    using a PIDLateralController. The vehicle follows the given trajectory and adjusts its
    steering angle to minimize cross-track error.
    """
    lateral_controller = PIDLateralController(K_P=0.3, K_D=0.2, K_I=0, dt=0.1)
    past_steer_angle = 0

    while True:
        cte = trajectory.signedDistanceTo(self.position)
        steering_angle = lateral_controller.run_step(cte)
        take RegulatedControlAction(throttle, steering_angle, past_steer_angle)
        past_steer_angle = steering_angle

behavior AdvBehavior():
    # Cross from right side before reaching the intersection node
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to intersection <= OPT_ADV_DISTANCE)
    do AccelerateBehavior(throttle=globalParameters.OPT_ADV_THROTTLE, trajectory=advTrajectoryLine) for globalParameters.OPT_ADV_TIMER seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary comes from the right (conflicting straight maneuver)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advTrajectoryLine = advManeuver.startLane.centerline + advManeuver.connectingLane.rightEdge + advManeuver.endLane.centerline
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

# Ensure adversary is on the right side of ego
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG
# Ego starts at a moderate distance from intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 40
# Adversary starts closer to intersection so it crosses before ego arrives
require 10 <= (distance from advSpawnPt to intersection) <= 20
