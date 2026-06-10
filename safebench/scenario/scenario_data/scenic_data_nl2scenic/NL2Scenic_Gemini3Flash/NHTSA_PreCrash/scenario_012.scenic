"""Scenario Description:

Vehicle is backing up in an urban area, in daylight, under clear weather conditions, 
at a driveway or alley location, with a posted speed limit of 25 mph; 
and collides with another vehicle.

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

WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Speed Limit for urban areas (25 mph is approx 11.17 m/s)
SPEED_LIMIT = 11.176 

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.a2'

#################################
# BEHAVIORS                     #
#################################

behavior BackingUpBehavior(target):
    """Behavior for the ego vehicle to back out of a driveway when an adversary is close."""
    # Stay stationary until the target vehicle approaches the 'driveway'
    while (distance to target) > 25:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    
    # Engage reverse gear and back into the road
    take SetReverseAction(True)
    take SetBrakeAction(0)
    while True:
        # Back up with moderate throttle to ensure collision
        take SetThrottleAction(0.6)
        take SetSteerAction(0.0)

behavior DrivingBehavior(speed):
    """Adversary drives along the lane at a constant speed."""
    do FollowLaneBehavior(target_speed=speed)

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Find a suitable urban lane with the specified speed limit
urban_lanes = [l for l in network.lanes if l.road.speedLimit and 10 <= l.road.speedLimit <= 13]
select_lane = Uniform(*urban_lanes)

# 2. Define a reference point on the road for spawning
spawn_pt = OrientedPoint on select_lane.centerline

# 3. Spawn the Ego vehicle at a 'driveway' location (offset from the road)
# We place it 8 meters to the left of the road and 15 meters ahead of the adversary's start
ego = new Car (left of spawn_pt by 8) offset along spawn_pt.heading by 15,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    facing spawn_pt.heading + 90 deg, # Facing away from the road
    with behavior BackingUpBehavior(adversary)

# 4. Spawn the Adversary vehicle on the main road
adversary = new Car at spawn_pt,
    with blueprint ADV_MODEL,
    with behavior DrivingBehavior(SPEED_LIMIT)

#################################
# REQUIREMENTS                  #
#################################

# Ensure the speed limit matches the scenario description
require 10 <= adversary.road.speedLimit <= 13

# Ensure the collision occurs by checking proximity
# We terminate the scenario once they are within a collision-likely distance
terminate when (distance to adversary) < 1.5