"""Scenario Description:

The ego car travels straight forward through the intersection. Meanwhile, an adversarial object enters from the right side after the intersection node.

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

param OPT_ADV_THROTTLE = Range(5, 10)/10
param OPT_ADV_TIMER = Range(10, 30)/10
param OPT_BRAKE_DISTANCE = Range(5, 8)  # Distance at which the ego vehicle begins to brake
param OPT_EGO_SPEED = Range(1, 5)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Range(8, 12)/10

# Trigger distance: adversarial starts entering when ego is this close to/past intersection
OPT_ADV_TRIGGER_DISTANCE = Range(0, 5)

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

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
    # Wait until ego has reached/passed the intersection node before entering
    do WaitBehavior() until (distance from ego to intersection) <= globalParameters.OPT_ADV_TRIGGER_DISTANCE
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to intersection <= 1)
    do AccelerateBehavior(throttle=globalParameters.OPT_ADV_THROTTLE, trajectory=advTrajectoryLine) for globalParameters.OPT_ADV_TIMER seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)

behavior WaitBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial enters from the right side (conflicting maneuver from right)
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
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

# Ensure adversarial is on the right side of ego
require CONST_MIN_RIGHT_DEG < (egoDir - advDir) < CONST_MAX_RIGHT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20