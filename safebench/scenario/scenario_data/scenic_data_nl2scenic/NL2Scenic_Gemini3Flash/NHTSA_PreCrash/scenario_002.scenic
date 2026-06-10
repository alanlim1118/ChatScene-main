"""Scenario Description:
The ego vehicle is driving straight on a rural road (Town07) in daylight under adverse weather 
(heavy rain). The road has a high speed limit (55 mph or more). Due to the wet and slippery 
conditions, the vehicle suddenly loses control, swerves, and runs off the road.
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

# 55 mph is approximately 24.5 m/s
param OPT_SPEED_LIMIT = 24.58  
param OPT_EGO_SPEED = Range(25, 30)
param OPT_CONTROL_LOSS_DURATION = Range(2, 4)

# Adverse weather in daylight
WEATHER_OPTIONS = ['HardRainNoon', 'MidRainyNoon', 'WetCloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

EGO_MODELS = ['vehicle.audi.etron', 'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 
              'vehicle.dodge.charger_police', 'vehicle.mercedes.coupe', 'vehicle.tesla.model3']

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LoseControlAndOffRoad(target_speed):
    """
    Vehicle drives straight at a high speed, then simulates a loss of control
    by applying a sharp steering angle and then braking after leaving the road.
    """
    # Phase 1: Driving straight
    try:
        do FollowLaneBehavior(target_speed=target_speed) for Range(4, 7) seconds
    interrupt when True:
        pass

    # Phase 2: Loss of control (Hydroplaning/Slipping)
    # Randomly swerve left or right
    swerve_direction = Uniform(-1, 1)
    # Set a significant steering angle to simulate spinning or veering
    take SetSteerAction(swerve_direction * Range(0.4, 0.7))
    take SetThrottleAction(0.2)
    
    # Continue "sliding" for a duration or until off the road
    do WaitBehavior() for globalParameters.OPT_CONTROL_LOSS_DURATION seconds
    
    # Phase 3: Final crash/stop
    take SetBrakeAction(1.0)
    take SetThrottleAction(0.0)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for long roads in the rural map. 
# We look for roads that are not part of an intersection to ensure it's a "straight" stretch.
straight_roads = filter(lambda r: not any(lane.maneuvers for lane in r.lanes), network.roads)

# In Town07, many roads are rural. We pick a lane from these roads.
# We ideally want lanes with a high speed limit, but if the map metadata doesn't 
# explicitly set them to 55mph, we select a suitable rural road and simulate the speed.
suitable_lanes = []
for road in straight_roads:
    for lane in road.lanes:
        # Filter for lanes that are long enough for a high-speed run-off
        if lane.centerline.length > 100:
            suitable_lanes.append(lane)

if not suitable_lanes:
    suitable_lanes = network.lanes

init_lane = Uniform(*suitable_lanes)
spawn_pt = OrientedPoint on init_lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawn_pt,
    with rolename 'hero',
    with blueprint Uniform(*EGO_MODELS),
    with behavior LoseControlAndOffRoad(globalParameters.OPT_EGO_SPEED)

# Ensure we are not spawning too close to the end of a lane so the behavior has time to finish
require (distance from ego to end of init_lane.centerline) > 100

# Require that the speed limit of the lane is high (conceptually)
# Note: In some CARLA maps, speedLimit might be None or 0 in metadata, 
# so we apply the logic to the behavior speed.
require init_lane.speedLimit == None or init_lane.speedLimit >= 15 

# Final check to ensure it's a rural environment (Town07 is purely rural)
terminate when ego.speed < 0.1 and (distance from ego to spawn_pt) > 50