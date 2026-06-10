"""Scenario Description:

Vehicle is changing lanes or passing in an urban area, in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph; and closes in on a lead vehicle.

The ego vehicle travels at the speed limit (approx 24.5 m/s) and encounters a slower lead vehicle.
It then performs a lane change to the adjacent lane to maintain its speed.

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

# 55 mph is approximately 24.58 m/s
SPEED_LIMIT = 24.58 
EGO_SPEED = SPEED_LIMIT
LEAD_SPEED = SPEED_LIMIT * 0.6  # Lead vehicle is significantly slower

# Distances
SPAWN_GAP = Range(30, 45)
LANE_CHANGE_TRIGGER_DIST = 20
BRAKE_DISTANCE = 8

# Weather
param weather = 'ClearNoon'

# Models
EGO_MODEL = 'vehicle.lincoln.mkz_2017'
carModels = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.bmw.grandtourer', 
             'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.dodge.charger_police', 
             'vehicle.mercedes.coupe', 'vehicle.mini.cooper_s', 'vehicle.nissan.micra', 
             'vehicle.nissan.patrol', 'vehicle.seat.leon', 'vehicle.tesla.model3', 
             'vehicle.toyota.prius', 'vehicle.volkswagen.t2']

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoPassingBehavior(target_speed, target_lane_sec):
    try:
        # Drive until close to the lead vehicle
        do FollowLaneBehavior(target_speed=target_speed) until withinDistanceToAnyObjs(self, LANE_CHANGE_TRIGGER_DIST)
        
        # Initiate lane change
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed)
        
        # Continue following the new lane
        do FollowLaneBehavior(target_speed=target_speed)

    interrupt when withinDistanceToAnyObjs(self, BRAKE_DISTANCE):
        # Emergency safety check
        take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent lane in the same direction
valid_lane_sections = []
for lane in network.lanes:
    for lane_sec in lane.sections:
        # Check if it's a forward lane and has an adjacent forward lane for passing
        if lane_sec.isForward:
            # Check left
            if lane_sec.laneToLeft and lane_sec.laneToLeft.isForward:
                valid_lane_sections.append((lane_sec, lane_sec.laneToLeft))
            # Or check right
            elif lane_sec.laneToRight and lane_sec.laneToRight.isForward:
                valid_lane_sections.append((lane_sec, lane_sec.laneToRight))

# Pick a random valid section from the filtered list
selected_pair = Uniform(*valid_lane_sections)
ego_start_lane_sec = selected_pair[0]
target_lane_sec = selected_pair[1]

# Define spawn points
ego_spawn_pt = new OrientedPoint in ego_start_lane_sec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Lead Vehicle ahead of Ego
lead_vehicle = new Car following roadDirection from ego_spawn_pt for SPAWN_GAP,
    with blueprint Uniform(*carModels),
    with behavior LeadVehicleBehavior(LEAD_SPEED)

# Spawn Ego Vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoPassingBehavior(EGO_SPEED, target_lane_sec)

# Ensure the scenario starts away from junctions as per description
require (distance to intersection) > 30
require (distance from lead_vehicle to intersection) > 30

# Terminate scenario after ego passes or moves a certain distance
terminate when distance from ego to ego_spawn_pt > 150