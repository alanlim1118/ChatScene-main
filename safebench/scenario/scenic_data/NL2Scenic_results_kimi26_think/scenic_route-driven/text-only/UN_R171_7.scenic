"""Scenario Description:

The ego vehicle follows a lead vehicle in a straight lane for over two seconds with a lateral offset of less than one meter, then ego vehicle successfully completes a full lane change maneuver involving a 3.5-meter lateral displacement into the adjacent lane.

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

LEAD_SPEED = 10
LEAD_DIST = 20

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
spawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawnPt

lead = new Car following roadDirection from ego for LEAD_DIST,
    with behavior FollowLaneBehavior(target_speed=LEAD_SPEED)

require (distance from ego to intersection) > 20
require (distance from lead to intersection) > 20
