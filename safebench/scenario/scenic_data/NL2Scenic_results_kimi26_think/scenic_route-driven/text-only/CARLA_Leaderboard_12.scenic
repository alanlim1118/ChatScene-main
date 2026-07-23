"""Scenario Description:

The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change into traffic moving in the same direction to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz_2017'

param OBSTACLE_DIST = Range(25, 40)
param ADJ_VEHICLE_OFFSET = Range(-5, 10)
param ADJACENT_SPEED = Range(4, 7)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToLeft

obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OBSTACLE_DIST

adjVehicleRef = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OBSTACLE_DIST + globalParameters.ADJ_VEHICLE_OFFSET
adjVehiclePos = adjLaneSec.centerline.project(adjVehicleRef.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL

Obstacle = new Car at obstacleSpawnPt,
    with heading ego.heading,
    with regionContainedIn egoLaneSec

AdjVehicle = new Car at adjVehiclePos,
    with heading ego.heading,
    with regionContainedIn adjLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADJACENT_SPEED)

require (distance to intersection) > 50
require (distance from Obstacle to intersection) > 50
require (distance from AdjVehicle to intersection) > 50
terminate when (distance to egoSpawnPt) > 120
