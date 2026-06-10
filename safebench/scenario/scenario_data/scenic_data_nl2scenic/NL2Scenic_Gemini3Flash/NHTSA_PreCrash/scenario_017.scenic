"""Scenario Description:

Vehicle (ego) is going straight in a rural area (Town07), in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph or more (approx. 25 m/s); 
and drifts/encroaches into another vehicle traveling in the opposite direction.

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

# 55 mph is approximately 24.58 m/s. We set the speeds at or above this.
param OPT_EGO_SPEED = Range(25, 30)
param OPT_ADV_SPEED = Range(23, 27)

# Weather: Clear daylight
param weather = 'ClearNoon'

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.etron'

# Distance at which the ego starts drifting into the opposite lane
DRIFT_TRIGGER_DISTANCE = Range(60, 80)

#################################
# AGENT BEHAVIORS               #
#################################

behavior OppositeVehicleBehavior(target_speed):
    """Behavior for the adversary vehicle to drive normally in its lane."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoDriftBehavior(target_speed, target_lane, adversary_obj):
    """
    Ego drives straight in its lane until it is close to the adversary, 
    then drifts into the opposite lane.
    """
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to adversary_obj) < DRIFT_TRIGGER_DISTANCE:
        # LaneChangeBehavior is used to simulate the encroachment/drift.
        # is_oppositeTraffic is True because the target lane is in the opposite direction.
        do LaneChangeBehavior(
            laneSectionToSwitchTo=target_lane.sections[0], 
            is_oppositeTraffic=True, 
            target_speed=target_speed
        )

#################################
# SPATIAL RELATIONS             #
#################################

# Select a road in the rural environment (Town07) that has opposite lanes
# and is not an intersection.
rural_roads = filter(lambda r: r.forwardLanes and r.backwardLanes, network.roads)
selected_road = Uniform(*rural_roads)

# Pick a forward lane for ego and the corresponding backward lane for the adversary
ego_lane = Uniform(*selected_road.forwardLanes.lanes)
adv_lane = Uniform(*selected_road.backwardLanes.lanes)

# Define spawn points. The adversary is placed further down the road in the opposite lane.
ego_spawn_pt = OrientedPoint on ego_lane.centerline
adv_spawn_pt = OrientedPoint on adv_lane.centerline, ahead of ego_spawn_pt by Range(150, 200)

#################################
# SCENARIO SPECIFICATION        #
#################################

# We spawn the adversary first so the ego's behavior can reference it
adversary = new Car at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with behavior OppositeVehicleBehavior(globalParameters.OPT_ADV_SPEED)

ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoDriftBehavior(globalParameters.OPT_EGO_SPEED, adv_lane, adversary)

#################################
# CONSTRAINTS                   #
#################################

# Ensure the scenario starts well away from junctions to satisfy the "non-junction" requirement
require (distance to intersection) > 50
require (distance from adversary to intersection) > 50

# Ensure the road is relatively straight for the initial part
require abs(relative heading of adversary from ego) > 170 deg

terminate when (distance to adversary) < 2 or (distance to ego_spawn_pt) > 300