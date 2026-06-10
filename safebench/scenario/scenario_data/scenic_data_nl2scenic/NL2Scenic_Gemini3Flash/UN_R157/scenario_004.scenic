"""Scenario Description:

The ego vehicle must maintain a stable position and remain within its lane while navigating a road with curvature. 
The scenario includes an adjacent vehicle driving close beside the ego in a neighboring lane and a leading vehicle 
(either a car or a motorcycle) that swerves within the same lane as the ego.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_CAR_MODELS = ['vehicle.audi.tt', 'vehicle.nissan.micra', 'vehicle.mini.cooper_s']
LEAD_MOTO_MODELS = ['vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf']

param OPT_EGO_SPEED = Range(8, 15)  # Testing across various speeds
param OPT_LEAD_DIST = Range(20, 30)
param OPT_WEAVE_AMPLITUDE = Range(0.4, 0.7)
param OPT_WEAVE_PERIOD = Range(8, 12)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WeavePIDBehavior(target_speed, weave_amplitude=0.3, weave_period=10):
    """
    Causes the vehicle to weave/swerve within its lane using a sine wave offset.
    """
    K_P, K_D, K_I = 0.2, 0.1, 0.01
    dt = 0.1
    pid = PIDLateralController(K_P, K_D, K_I, dt)
    past_steer = 0.0

    while True:
        # Get current lane centerline
        trajectoryLine = self.laneSection.centerline
        proj = trajectoryLine.project(self.position)
        progress = distance from trajectoryLine[0] to proj

        # Calculate cross-track error to a swerving path
        sine_offset = weave_amplitude * sin(progress / weave_period)
        cte = trajectoryLine.signedDistanceTo(self.position) - sine_offset

        steer = pid.run_step(cte)
        take RegulatedControlAction(target_speed, steer, past_steer)
        past_steer = steer

behavior MatchSpeedBehavior(reference_actor):
    """
    Follows the lane while maintaining the speed of the reference actor to stay beside it.
    """
    while True:
        target_speed = reference_actor.speed if reference_actor.speed is not None else 10
        do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lane sections that have at least one adjacent forward lane for the side vehicle
laneSecsWithAdj = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            # Check if there's a forward lane on either side
            has_adj = (laneSec._laneToLeft and laneSec._laneToLeft.isForward) or \
                      (laneSec._laneToRight and laneSec._laneToRight.isForward)
            if has_adj:
                laneSecsWithAdj.append(laneSec)

# Selection of the ego starting point
egoLaneSec = Uniform(*laneSecsWithAdj)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Determine which side lane to use for the adjacent vehicle
if egoLaneSec._laneToLeft and egoLaneSec._laneToLeft.isForward:
    sideLaneSec = egoLaneSec._laneToLeft
else:
    sideLaneSec = egoLaneSec._laneToRight

# Define spawn points for lead and side vehicles
SideSpawnPt = new OrientedPoint in sideLaneSec.centerline,
    at sideLaneSec.centerline.project(egoSpawnPt.position)

LeadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# The leading vehicle: can be a passenger car or a motorcycle
lead_blueprint = Uniform(*(LEAD_CAR_MODELS + LEAD_MOTO_MODELS))
LeadAgent = new CarlaActor at LeadSpawnPt,
    with blueprint lead_blueprint,
    with behavior WeavePIDBehavior(
        target_speed=globalParameters.OPT_EGO_SPEED,
        weave_amplitude=globalParameters.OPT_WEAVE_AMPLITUDE,
        weave_period=globalParameters.OPT_WEAVE_PERIOD
    )

# The ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

# The adjacent vehicle driving close beside the ego
SideAgent = new Car at SideSpawnPt,
    with behavior MatchSpeedBehavior(ego)

# Ensure we are not starting immediately at an intersection to allow for curvature navigation
require distance to intersection >= 50
require distance to intersection <= 500

# Terminate after a duration of stable driving
terminate after 30 seconds