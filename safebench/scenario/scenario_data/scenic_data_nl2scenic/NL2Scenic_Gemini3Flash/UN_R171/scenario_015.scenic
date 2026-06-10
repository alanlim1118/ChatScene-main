"""Scenario Description:

The ego vehicle maintains its straight path while approaching a slower moving vehicle target ahead. 
The target vehicle is intentionally positioned with a significant lateral offset from the ego’s centerline 
(exceeding the standard 0.5-meter threshold) while remaining within the same lane.

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

# Speed constants
param EGO_SPEED = Range(12, 15)
param LEAD_SPEED = Range(5, 8)

# Distance and Offset constants
param INITIAL_DISTANCE = Range(30, 45)
# Define lateral offset magnitude to be between 0.6m and 1.0m (greater than 0.5m)
# We sample a sign to decide left or right offset
param OFFSET_SIDE = Uniform(-1, 1)
param OFFSET_MAGNITUDE = Range(0.6, 0.9)
OFFSET_VAL = globalParameters.OFFSET_MAGNITUDE * globalParameters.OFFSET_SIDE

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior LeadVehicleBehavior(target_speed):
    # The lead vehicle drives at a constant slower speed
    # We use FollowLaneBehavior which generally tracks the lane
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that are long enough and on a road (not an intersection)
# We prefer straight sections for this scenario
straight_lanes = filter(lambda l: not l.road.is_connecting and any(isinstance(m.type, ManeuverType) and m.type == ManeuverType.STRAIGHT for m in l.maneuvers), network.lanes)

assert len(list(straight_lanes)) > 0, "No suitable straight lanes found in this map."

ego_lane = Uniform(*straight_lanes)

# Position ego on the centerline
ego_spawn_pt = OrientedPoint on ego_lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Position the lead vehicle ahead of ego
lead_center_pt = OrientedPoint following roadDirection from ego for globalParameters.INITIAL_DISTANCE

# Apply the lateral offset
# We use 'left of' with our signed OFFSET_VAL
lead_spawn_pt = lead_center_pt offset by (OFFSET_VAL @ 0)

lead_vehicle = new Car at lead_spawn_pt,
    with behavior LeadVehicleBehavior(globalParameters.LEAD_SPEED)

#################################
# CONSTRAINTS                   #
#################################

# 1. Ensure lead vehicle is in the same lane as ego
require lead_vehicle in ego_lane

# 2. Ensure lead vehicle is on the road and not in an intersection during setup
require not lead_vehicle.road.is_connecting
require not ego.road.is_connecting

# 3. Explicitly verify the lateral offset exceeds 0.5m from the centerline
# distance to a polyline returns the perpendicular distance
require (distance from lead_vehicle to ego_lane.centerline) > 0.5

# 4. Ensure there's enough room to complete the approach
require (distance to intersection) > 80

# Terminate when ego has passed the lead vehicle or after some distance
terminate when (distance from ego to ego_spawn_pt) > 100