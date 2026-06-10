"""Scenario Description:

A stopped vehicle, A, was looking left and right down a cross road waiting for traffic to clear before proceeding. 
Another driver, B (ego), waiting behind A was also checking crossing traffic. 
Vehicle A started to go, decided that it wasn't safe, and abruptly stopped. 
Driver B, who had been watching traffic, thought that A had moved on and proceeded, rear-ending driver A.

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
LEAD_CAR_MODEL = "vehicle.audi.tt"

# Speeds and timings
param LEAD_START_SPEED = 5
param EGO_FOLLOW_SPEED = 8
param LEAD_MOVE_DURATION = Range(1.2, 1.8)
param INITIAL_GAP = Range(6, 8)
param REACTION_DELAY = Range(0.5, 1.0)

#################################
# AGENT BEHAVIORS               #
#################################

# Behavior for Vehicle A: Starts, then stops abruptly
behavior LeadCarBehavior():
    # 1. Initial wait at the cross road
    take SetBrakeAction(1.0)
    wait Range(2, 4) seconds
    
    # 2. Start to proceed
    try:
        do FollowLaneBehavior(target_speed=globalParameters.LEAD_START_SPEED) for globalParameters.LEAD_MOVE_DURATION seconds
    
    # 3. Abruptly stop (deciding it's unsafe)
    finally:
        take SetThrottleAction(0)
        while True:
            take SetBrakeAction(1.0)

# Behavior for Vehicle B (Ego): Follows Lead Car but fails to brake when Lead stops
behavior EgoRearEndBehavior(lead_vehicle):
    # 1. Wait behind A
    while (distance from self to lead_vehicle) < (globalParameters.INITIAL_GAP + 0.5):
        take SetBrakeAction(1.0)
    
    # 2. Reaction delay (simulating checking traffic and thinking A has left)
    wait globalParameters.REACTION_DELAY seconds
    
    # 3. Proceed forward (distracted from lead car's stop)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_FOLLOW_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a 4-way non-signalized intersection for the cross road
intersections = filter(lambda i: i.is4Way and not i.isSignalized, network.intersections)
selected_intersection = Uniform(*intersections)
incoming_lane = Uniform(*selected_intersection.incomingLanes)

# Positioning points
# Lead car at the stop line of the intersection
lead_car_spawn_pt = incoming_lane.centerline.end
# Ego car behind the lead car
ego_spawn_pt = lead_car_spawn_pt offset along incoming_lane.orientation by -globalParameters.INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawning Ego (Vehicle B)
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    facing incoming_lane.orientation,
    with blueprint EGO_MODEL

# Spawning Lead Car (Vehicle A)
leadCar = new Car at lead_car_spawn_pt,
    facing incoming_lane.orientation,
    with blueprint LEAD_CAR_MODEL,
    with behavior LeadCarBehavior()

# Assign the behavior to ego now that leadCar is defined
ego.behavior = EgoRearEndBehavior(leadCar)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure they are at the intersection
require (distance to selected_intersection) < 10

# Terminate when the rear-end happens or after sufficient time
terminate when (distance to leadCar) < 0.1
terminate after 15 seconds