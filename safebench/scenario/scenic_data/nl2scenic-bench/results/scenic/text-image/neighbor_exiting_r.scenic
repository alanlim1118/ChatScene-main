"""Scenario Description:

In a top-down schematic view of a multi-lane roadway, a blue ego vehicle travels straight forward in the upper lane, indicated by a straight horizontal arrow pointing to the right. Positioned in the adjacent lane below, a pink adversarial vehicle executes a maneuver to steer away towards the right, depicted by a curved pink arrow that sweeps downward and to the right, signifying an object exiting from the right side of the ego vehicle's path.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(6, 10)
param OPT_ADV_STEER_AMPLITUDE = Range(2.5, 4.0)  # Lateral offset amplitude for rightward exit
param OPT_ADV_STEER_PERIOD = Range(15, 25)       # Spatial period of the rightward curve
param OPT_ADV_TRIGGER_DIST = Range(10, 20)       # Distance at which adv begins steering right

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < 5):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior SteerRightBehavior(target_speed, amplitude, period):
    """
    Adversarial vehicle steers away to the right using a smooth lateral offset
    relative to its current lane centerline, simulating an exit maneuver.
    """
    K_P = 0.3
    K_D = 0.15
    K_I = 0.02
    dt = 0.1
    pid = PIDLateralController(K_P, K_D, K_I, dt)
    pid.windup_guard = 0.6
    past_steer = 0.0

    while True:
        trajectoryLine = self.laneSection.centerline
        proj = trajectoryLine.project(self.position)
        progress = distance from trajectoryLine[0] to proj

        # Positive sine offset moves vehicle to the right of centerline
        right_offset = amplitude * (1 - cos(progress / period))
        cte = trajectoryLine.signedDistanceTo(self.position) - right_offset

        steer = pid.run_step(cte)
        take RegulatedControlAction(target_speed, steer, past_steer)
        past_steer = steer
        wait

behavior AdvBehavior(speed, amplitude, period, trigger_dist):
    # Initially follow lane normally, then steer right when close enough to ego
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego < trigger_dist)
    do SteerRightBehavior(speed, amplitude, period)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right neighbor (ego is in upper/left lane, adv in lower/right lane)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

require len(laneSecsWithRightLane) > 0

egoLaneSec = Uniform(*laneSecsWithRightLane)
adjLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversarial vehicle in the adjacent right lane, roughly alongside or slightly behind ego
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advOffset = Uniform(-5, 5)  # Slight longitudinal variation
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for advOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in the upper (left) lane, traveling straight
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),  # Blue
    with behavior EgoBehavior()

# Adversarial vehicle in the adjacent lower (right) lane, steering right to exit
AdvAgent = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),  # Pink
    with behavior AdvBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_ADV_STEER_AMPLITUDE,
        globalParameters.OPT_ADV_STEER_PERIOD,
        globalParameters.OPT_ADV_TRIGGER_DIST
    )

require distance from egoSpawnPt to intersection >= 80