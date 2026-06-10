"""Scenario Description:

The ego vehicle travels on a straight highway section behind a passenger car. 
The lead vehicle suddenly performs a high-intensity emergency braking maneuver 
until it reaches a complete stop. The ego vehicle must respond by braking 
to maintain a safe distance and avoid a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town06 is a highway-focused map with long straight sections
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Blueprints
EGO_MODEL = "vehicle.tesla.model3"
LEAD_VEHICLE_MODEL = "vehicle.audi.tt"

# Speed and Timing
HIGHWAY_SPEED = Range(20, 25)           # Approx 70-90 km/h
EMERGENCY_BRAKE_TRIGGER_TIME = Range(4, 7) # Seconds before lead vehicle brakes
EGO_BRAKING_THRESHOLD = 20              # Distance (m) to trigger ego's emergency brake

# Initial Spatial Gap
INITIAL_DISTANCE = Range(20, 25)

#################################
# AGENT BEHAVIORS               #
#################################

# LEAD VEHICLE BEHAVIOR: Drive at highway speed then slam on brakes
behavior LeadVehicleEmergencyBrakeBehavior(target_speed):
    # Drive normally for a few seconds
    do FollowLaneBehavior(target_speed=target_speed) for EMERGENCY_BRAKE_TRIGGER_TIME seconds
    
    # Perform emergency braking until stop
    while self.speed > 0.1:
        take SetThrottleAction(0), SetBrakeAction(1.0)
    
    # Stay stopped
    while True:
        take SetBrakeAction(1.0)

# EGO BEHAVIOR: Drive and respond to the lead vehicle's deceleration
behavior EgoBrakingBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    
    interrupt when withinDistanceToObjsInLane(self, EGO_BRAKING_THRESHOLD):
        # Apply full brakes to avoid collision
        while self.speed > 0:
            take SetThrottleAction(0), SetBrakeAction(1.0)
        # Keep brakes on once stopped
        while True:
            take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for highway lanes (Town06 lanes are generally suitable)
highway_lane = Uniform(*network.lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn lead vehicle
lead_vehicle = new Car on highway_lane.centerline,
    with blueprint LEAD_VEHICLE_MODEL,
    with behavior LeadVehicleEmergencyBrakeBehavior(HIGHWAY_SPEED)

# Spawn ego vehicle behind the lead vehicle
ego = new Car following roadDirection from lead_vehicle for -INITIAL_DISTANCE,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBrakingBehavior(HIGHWAY_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we are on a long straight stretch of the highway
require (distance to intersection) > 100
# Ensure both vehicles start in the same lane section
require lead_vehicle.laneSection == ego.laneSection

# Terminate scenario when both vehicles have come to a complete stop
terminate when ego.speed < 0.1 and lead_vehicle.speed < 0.1 and simulation().currentTime > EMERGENCY_BRAKE_TRIGGER_TIME