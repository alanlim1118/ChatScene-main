"""Scenario Description:

The ego vehicle travels at a constant speed in a straight lane while a pedestrian target emerges from the side 
and crosses perpendicular to the vehicle's path at a steady 5 km/h, timed so that the target would collide 
with the vehicle's longitudinal centerline if no intervention occurs, requiring the ego vehicle to 
detect the pedestrian at least 4 seconds before impact and execute an autonomous braking response 
to avoid a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Speeds
param OPT_EGO_SPEED = 10.0  # m/s (~36 km/h)
param OPT_PED_SPEED = 5 / 3.6  # 5 km/h converted to m/s (~1.388 m/s)

# Distances/Timing
param OPT_BRAKE_TIME_THRESHOLD = 4.0  # Seconds before impact to initiate braking
param OPT_PED_SIDE_OFFSET = 12.0      # How far the pedestrian starts from the centerline

# Calculate the time it takes for the pedestrian to reach the centerline
# Time = Distance / Speed
PED_REACH_CENTER_TIME = globalParameters.OPT_PED_SIDE_OFFSET / globalParameters.OPT_PED_SPEED

# Calculate the distance the ego must be from the crossing point when the pedestrian starts
# to ensure a collision would occur at the centerline
# Trigger Dist = Ego Speed * Time
COLLISION_TRIGGER_DIST = globalParameters.OPT_EGO_SPEED * PED_REACH_CENTER_TIME

# Calculate the distance at which the ego should detect and brake
# Brake Dist = Ego Speed * 4 seconds
EGO_BRAKE_DIST = globalParameters.OPT_EGO_SPEED * globalParameters.OPT_BRAKE_TIME_THRESHOLD

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedestrianBehavior(ego_actor, trigger_dist, collision_pt):
    """Wait until the ego is at the correct distance to ensure collision, then walk across."""
    wait until (distance from ego_actor to collision_pt) <= trigger_dist
    do WalkForwardBehavior(speed=globalParameters.OPT_PED_SPEED)

behavior EgoBehavior(target_speed, brake_dist, collision_pt):
    """Drive at constant speed and brake when the 4-second impact threshold is reached."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to collision_pt) <= brake_dist:
        # Autonomous braking response
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        while True:
            wait # Stay stopped

#################################
# SPATIAL RELATIONS             #
#################################

# Find a straight road segment (not in an intersection)
lane = Uniform(*filter(lambda l: not l.intersection, network.lanes))

# Define spawn point for Ego
egoSpawnPt = new OrientedPoint in lane.centerline

# Define the collision point (the spot on the lane the ego will pass)
# We place it far enough ahead to allow for the triggers to fire
crossPt = new OrientedPoint following roadDirection from egoSpawnPt for 120

# Define spawn point for the Pedestrian (to the left of the ego's path)
pedSpawnPt = new OrientedPoint left of crossPt by globalParameters.OPT_PED_SIDE_OFFSET,
    facing (crossPt.heading + 90 deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED, 
        EGO_BRAKE_DIST, 
        crossPt
    )

# Spawn crossing Pedestrian
ped = new Pedestrian at pedSpawnPt,
    with behavior PedestrianBehavior(
        ego, 
        COLLISION_TRIGGER_DIST, 
        crossPt
    )

#################################
# CONSTRAINTS                   #
#################################

# Ensure the ego has enough room to reach the trigger distances
require (distance from egoSpawnPt to crossPt) > COLLISION_TRIGGER_DIST + 10
require (distance from crossPt to intersection) > 20

# Terminate if the ego successfully avoids the pedestrian and stops
terminate when ego.speed < 0.1 and (distance from ego to crossPt) < EGO_BRAKE_DIST