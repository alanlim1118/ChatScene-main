"""Scenario Description:

A top-down schematic view illustrates a traffic scenario on a paved road with dashed white lane markings where a blue ego vehicle is positioned on the left side of the lane, traveling straight forward towards the right. Directly ahead in the same lane, a pink adversarial vehicle is positioned facing the same direction but is reversing backward, indicated by a left-pointing arrow, moving directly toward the approaching blue car. The scenario depicts a potential collision course as the forward-moving ego vehicle closes the distance to the reversing object ahead.

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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(4, 7)        # Ego moving forward at moderate speed
param OPT_ADV_REVERSE_SPEED = Range(2, 5) # Adversary reversing toward ego
param OPT_BRAKE_DISTANCE = Range(8, 12)   # Distance at which ego brakes
param OPT_INITIAL_GAP = Range(25, 40)     # Initial distance between ego and adversary

CONST_SAME_DIR_TOL = 15 deg              # Tolerance for same-direction heading alignment

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

behavior ReverseBehavior(target_speed=3):
    """
    Drives in reverse along the lane centerline by following the trajectory
    backwards. Uses negative throttle to achieve reverse motion.
    """
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0
    
    # Build reversed trajectory for lateral control reference
    reversed_traj = list(reversed(advTrajectory))
    reversed_centerline = concatenateCenterlines([lane.centerline for lane in reversed_traj])
    
    while True:
        current_speed = self.speed if self.speed is not None else 0
        cte = reversed_centerline.signedDistanceTo(self.position)
        
        # Negative speed error drives reverse throttle
        speed_error = -target_speed - current_speed
        throttle = _lon_controller.run_step(speed_error)
        current_steer_angle = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment (not at an intersection)
roadSegment = Uniform(*filter(lambda s: not s.isIntersection and len(s.lanes) >= 1, network.roads))
lane = Uniform(*roadSegment.lanes)

# Ego spawn point on the left side of the lane
egoSpawnPt = new OrientedPoint in lane.leftEdge

# Adversary spawn point ahead of ego in the same lane, facing same direction
advSpawnPt = new OrientedPoint following lane.orientation from egoSpawnPt for globalParameters.OPT_INITIAL_GAP

# Trajectories for both vehicles along the same lane
egoTrajectory = [lane]
advTrajectory = [lane]

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),           # Blue ego vehicle
    with regionContainedIn None,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,  # Facing same direction as ego
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),         # Pink adversarial vehicle
    with regionContainedIn None,
    with behavior ReverseBehavior(target_speed=globalParameters.OPT_ADV_REVERSE_SPEED)

# Ensure both vehicles face approximately the same direction
require abs(egoDir - advDir) < CONST_SAME_DIR_TOL

# Ensure adversary is actually ahead of ego along the lane
require (distance from egoSpawnPt to advSpawnPt) >= 15