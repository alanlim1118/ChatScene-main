"""Scenario Description:
Vehicle is leaving a parked position at night, in an urban area, under clear weather conditions, 
at a non-junction location with a posted speed limit of 25 mph; 
and collides with an object on road shoulder or parking lane.
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

# 25 mph is approximately 11.17 meters per second
TARGET_SPEED = 11.17
EGO_MODEL = 'vehicle.audi.tt'

# Using ClearSunset to represent clear conditions at the beginning of the night
WEATHER_OPTIONS = ['ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeavesParking(target_speed):
    """Ego starts moving forward and then steers slightly to leave the parked position."""
    try:
        # Move forward slowly in the parking/shoulder area
        take SetThrottleAction(0.4) for 1.5 seconds
        
        # Initiate a slight steer to transition into the main driving lane
        take SetSteerAction(-0.1), SetThrottleAction(0.3)
        
        # Follow the lane normally thereafter
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when self.speed < 0.1 and simulation().currentTime > 10:
        # Stop if we hit the object and come to a halt
        terminate

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Select a suitable urban road that is not an intersection and has a curb
# Town10HD provides a downtown urban environment.
potentialLanes = filter(lambda l: not l.road.intersection and l.group.curb is not None, network.lanes)
lane = Uniform(*potentialLanes)

# 2. Define the starting position near the curb
# We sample a point on the curb region of the selected lane group
curb_pt = new OrientedPoint on lane.group.curb

# 3. Spawn ego vehicle
# We place the car 1.0m to the left of the curb point (towards the road center)
# to simulate being in a parked/shoulder position parallel to the road.
ego = new Car left of curb_pt by 1.0,
    with rolename 'hero',
    facing lane.heading,
    with blueprint EGO_MODEL,
    with behavior EgoLeavesParking(TARGET_SPEED)

# 4. Spawn the object on the shoulder/parking lane path
# We place a trash bin/object ahead along the same curb line.
# If the ego is 1.0m from the curb, placing the object 0.5m from the curb puts it 
# in the path of the vehicle's right side.
target_curb_pt = new OrientedPoint on lane.group.curb,
    beyond curb_pt by Range(5, 10)

obstacle = new Trash left of target_curb_pt by 0.5,
    with heading target_curb_pt.heading

#################################
# CONSTRAINTS                   #
#################################

# Ensure the lane speed limit matches the "posted 25 mph" requirement (~11.17 m/s)
# We allow a small range as CARLA maps have discrete speed limit values.
require 8 <= lane.speedLimit <= 13

# Safety requirement to ensure the object is spawned at a reasonable distance
require distance from ego to obstacle > 4

# Termination condition
terminate after 20 seconds