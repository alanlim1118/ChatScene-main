"""Scenario Description:

Vehicle is backing up in an urban area (Town10HD), in daylight, under clear weather conditions, 
with a posted speed limit of 25 mph (approx 11.17 m/s). 
The vehicle then departs the road edge on the shoulder/parking lane in a driveway/alley location.

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
# 25 mph is approximately 11.17 m/s
SPEED_LIMIT = 11.17 

# Weather setup
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior BackingUpAndDepartBehavior(target_speed):
    """
    Behavior for backing up from a road edge/driveway position 
    and then departing forward into the main lane.
    """
    # 1. Backing Up Phase
    take SetReverseAction(True)
    # Apply a gentle throttle to back up
    take SetThrottleAction(0.3)
    do WaitBehavior() for 4 seconds
    
    # 2. Stop and shift gear
    take SetThrottleAction(0)
    take SetBrakeAction(1.0)
    do WaitBehavior() for 1 second
    take SetBrakeAction(0)
    take SetReverseAction(False)
    
    # 3. Depart road edge/shoulder
    # FollowLaneBehavior naturally handles steering from an offset back to the lane center
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a road in the urban environment
# We look for a lane that likely has a shoulder or parking edge (road edge)
selected_lane = Uniform(*network.lanes)

# Place the ego vehicle at the edge of the lane (simulating a shoulder/parking spot/driveway edge)
# We calculate an offset to the right side of the lane
road_edge_offset = (selected_lane.width / 2) - 0.5 

# We pick a point along the lane
spawn_pt = new OrientedPoint on selected_lane.centerline,
    offset by road_edge_offset @ 0

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior BackingUpAndDepartBehavior(SPEED_LIMIT)

# Ensure the scenario starts in an urban street section away from immediate intersection clutter
require distance to intersection > 30

# Terminate after the vehicle has successfully departed and driven for a while
terminate when ego.speed > 5 and (distance to spawn_pt) > 50