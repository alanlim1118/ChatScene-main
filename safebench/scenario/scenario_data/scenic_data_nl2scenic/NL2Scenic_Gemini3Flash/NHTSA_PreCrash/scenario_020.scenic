"""Scenario Description:
Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph or more; and closes in on a lead 
vehicle moving at lower constant speed.
"""

#################################
# MAP AND MODEL                 #
#################################

# Town06 is selected as it contains long highway sections with high speed limits 
# suitable for the 55 mph (approx. 88.5 km/h) requirement.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 55 mph in meters per second
SPEED_LIMIT_THRESHOLD = 24.58

# Sampling a car model from the provided list
carModels = [
    'vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt', 
    'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 
    'vehicle.dodge.charger_police', 'vehicle.jeep.wrangler_rubicon', 
    'vehicle.lincoln.mkz_2017', 'vehicle.mercedes.coupe', 'vehicle.mini.cooper_s', 
    'vehicle.ford.mustang', 'vehicle.nissan.micra', 'vehicle.nissan.patrol', 
    'vehicle.seat.leon', 'vehicle.tesla.model3', 'vehicle.toyota.prius', 
    'vehicle.volkswagen.t2'
]

param EGO_MODEL = Uniform(*carModels)
param LEAD_MODEL = Uniform(*carModels)

# Weather and time of day (Daylight and Clear conditions)
param weather = 'ClearNoon'

# Speed parameters: Ego speed >= 55 mph (24.58 m/s), Lead speed lower
param OPT_EGO_SPEED = Range(25, 30) 
param OPT_LEAD_SPEED = Range(15, 20)
param OPT_INITIAL_DISTANCE = Range(40, 60)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_speed):
    """Lead vehicle drives at a constant lower speed."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoVehicleBehavior(target_speed):
    """Ego vehicle drives at a higher speed, closing the gap until it needs to avoid collision."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, 15):
        # Once closing in, maintain safety distance or perform avoidance
        do DriveAvoidingCollisions(target_speed=target_speed, avoidance_threshold=15)

#################################
# SPATIAL RELATIONS             #
#################################

# Filtering for lanes that have a high posted speed limit (>= 55 mph)
# and are not part of an intersection to satisfy the "non-junction" requirement.
high_speed_lanes = filter(lambda l: l.road.speedLimit >= SPEED_LIMIT_THRESHOLD, network.lanes)

# Select one of these lanes for the scenario
selected_lane = Uniform(*high_speed_lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego vehicle on the selected high-speed lane
ego = new Car on selected_lane.centerline,
    with rolename 'hero',
    with blueprint globalParameters.EGO_MODEL,
    with behavior EgoVehicleBehavior(globalParameters.OPT_EGO_SPEED)

# Spawn Lead vehicle ahead of the Ego vehicle in the same lane
lead_car = new Car ahead of ego by globalParameters.OPT_INITIAL_DISTANCE,
    with blueprint globalParameters.LEAD_MODEL,
    with behavior LeadVehicleBehavior(globalParameters.OPT_LEAD_SPEED)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure vehicles start on a straight road section away from junctions
require not ego.intersection
require not lead_car.intersection
require (distance to intersection) > 50

# The scenario is successful once the ego vehicle has closed the distance
terminate when distance to lead_car < 18