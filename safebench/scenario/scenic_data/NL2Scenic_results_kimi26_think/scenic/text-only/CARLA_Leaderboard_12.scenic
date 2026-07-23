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

param EGO_SPEED = Range(5, 8)
param OBSTACLE_DIST = Range(25, 40)
param ADJ_VEHICLE_OFFSET = Range(-5, 10)
param ADJACENT_SPEED = Range(4, 7)
param LANE_CHANGE_TRIGGER = Range(12, 18)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to Obstacle) < globalParameters.LANE_CHANGE_TRIGGER:
        do LaneChangeBehavior(laneSectionToSwitch=adjLaneSec, target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithAdjacentLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithAdjacentLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithAdjacentLane)
adjLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OBSTACLE_DIST

adjVehicleRef = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OBSTACLE_DIST + globalParameters.ADJ_VEHICLE_OFFSET
adjVehiclePos = adjLaneSec.centerline.project(adjVehicleRef.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

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