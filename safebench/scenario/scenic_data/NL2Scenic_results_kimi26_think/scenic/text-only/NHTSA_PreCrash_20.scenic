"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph or more; and closes in on a lead vehicle moving at lower constant speed

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Speeds in m/s (55 mph ≈ 24.6 m/s)
param OPT_EGO_SPEED = Range(26, 30)       # Ego speed above 55 mph equivalent
param OPT_LEAD_SPEED = Range(10, 15)      # Lead vehicle lower constant speed
param OPT_INITIAL_DISTANCE = Range(50, 80) # Initial distance between ego and lead

#################################
# AGENT BEHAVIORS               #
#################################

behavior ConstantSpeedBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment (non-junction) and one of its lanes
road = Uniform(*network.roads)
lane = Uniform(*road.lanes)

# Spawn lead vehicle ahead on the lane
leadSpawnPt = new OrientedPoint on lane.centerline

# Spawn ego behind the lead vehicle along the same lane
egoSpawnPt = new OrientedPoint following lane.orientation from leadSpawnPt for -globalParameters.OPT_INITIAL_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

leadVehicle = new Car at leadSpawnPt,
    with behavior ConstantSpeedBehavior(globalParameters.OPT_LEAD_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with behavior ConstantSpeedBehavior(globalParameters.OPT_EGO_SPEED)

# Terminate when the ego vehicle has closed in on the lead vehicle
terminate when distance from ego to leadVehicle < 15