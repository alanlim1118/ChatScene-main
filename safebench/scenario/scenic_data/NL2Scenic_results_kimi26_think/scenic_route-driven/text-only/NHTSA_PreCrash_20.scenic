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

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
lane = network.laneAt(egoSpawnPt.position)

# Spawn lead vehicle ahead of ego by the same distance the original placed ego behind it
leadSpawnPt = new OrientedPoint following lane.orientation from egoSpawnPt for globalParameters.OPT_INITIAL_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

leadVehicle = new Car at leadSpawnPt,
    with behavior ConstantSpeedBehavior(globalParameters.OPT_LEAD_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn None

# Terminate when the ego vehicle has closed in on the lead vehicle
terminate when distance from ego to leadVehicle < 15
