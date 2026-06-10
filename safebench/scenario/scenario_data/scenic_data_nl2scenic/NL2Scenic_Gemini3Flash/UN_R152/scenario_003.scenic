"""Scenario Description:

The subject vehicle (ego) drives along a small radius curved road. 
Guard pipes are constructed on the outer side of the curve. 
A stationary target (a car, pedestrian, or bicycle) is positioned just outside the guard pipes, 
aligned with the extension of the center of the lane (the tangent of the lane's centerline).

"""

#################################
# MAP AND MODEL                 #
#################################

# Town04 is selected for its mountainous, curved road sections which fit the 'small radius' description.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Ego vehicle properties
EGO_MODEL = "vehicle.lincoln.mkz_2017"
param EGO_SPEED = Range(7, 12)

# Weather setup
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Stationary target types: M1 category car, pedestrian, or bicycle
param target_class = Uniform(Car, Pedestrian, Bicycle)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, 10):
        # Emergency brake if the stationary object is detected
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
        while True:
            wait

behavior StationaryBehavior():
    # Target remains stationary
    while True:
        take SetSpeedAction(0)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for a lane that is part of a road (not an intersection) and has a significant curve.
# We look for lanes where the centerline length is notably greater than the direct distance between start and end.
curved_lanes = filter(lambda l: l.road and not l.intersection and l.centerline.length > 30, network.lanes)
sharp_curves = filter(lambda l: l.centerline.length > l.centerline.start.distanceTo(l.centerline.end) * 1.12, curved_lanes)

# Select a lane from the available sharp curves
target_lane = Uniform(*sharp_lanes) if sharp_lanes else Uniform(*curved_lanes)

# Identify the 'outer side' of the curve. In a curve, the outer edge is longer than the inner edge.
is_right_outer = target_lane.rightEdge.length > target_lane.leftEdge.length
outer_edge = target_lane.rightEdge if is_right_outer else target_lane.leftEdge

# Start point and heading for the extension
ego_spawn_pt = target_lane.centerline.start
extension_heading = target_lane.centerline.headingAt(0)

# Target position is on the extension of the center of the lane.
# We project along the starting tangent. As the road curves, this line will cross the outer edge.
param target_dist = Range(25, 35)
target_pos = ego_spawn_pt offset along extension_heading by globalParameters.target_dist

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Spawn Ego Vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    facing extension_heading,
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior(globalParameters.EGO_SPEED)

# 2. Spawn Guard Pipes (Barriers)
# We place several barriers along the outer edge to represent the guard pipes.
for i in range(12):
    # Place barriers starting a bit into the curve
    new Barrier on outer_edge.interpolate(i * 2.5 + 5),
        facing (outer_edge.headingAt(i * 2.5 + 5) + 90 deg)

# 3. Spawn the Stationary Target
# Positioned at the extension of the center, just outside the pipes.
stationary_target = new (globalParameters.target_class) at target_pos,
    facing extension_heading,
    with behavior StationaryBehavior()

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure the target is actually outside the road boundaries (due to the curve)
require stationary_target not in target_lane.road.region
# Ensure the target is within a reasonable distance to the ego for the scenario to trigger
require 20 < (distance to stationary_target) < 45

# Terminate when the ego vehicle has successfully stopped or passed the event
terminate when (distance to stationary_target) < 2 or ego.speed < 0.1 and (distance to stationary_target) < 15