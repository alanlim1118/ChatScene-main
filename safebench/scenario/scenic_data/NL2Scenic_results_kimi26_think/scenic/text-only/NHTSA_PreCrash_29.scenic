"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, at a non-junction location with a posted speed limit of 35 mph; and takes an evasive action to avoid an obstacle.

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

MODEL = 'vehicle.lincoln.mkz_2017'

# 35 mph ≈ 15.65 m/s
param EGO_SPEED = VerifaiRange(14, 16)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param SAFETY_DIST = VerifaiRange(15, 25)
OBSTACLE_DIST = Uniform(40, 70)
TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, 3):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a long road segment away from intersections
road = Uniform(*filter(lambda r: r.length > 150, network.roads))
lane = Uniform(*road.lanes)

# Spawn point for ego on the selected lane
egoSpawnPt = new OrientedPoint on lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior([lane])

# Stationary obstacle placed ahead on the same lane
obstacle = new Car following roadDirection from egoSpawnPt for OBSTACLE_DIST,
    with blueprint MODEL

terminate when (distance to egoSpawnPt) > TERM_DIST