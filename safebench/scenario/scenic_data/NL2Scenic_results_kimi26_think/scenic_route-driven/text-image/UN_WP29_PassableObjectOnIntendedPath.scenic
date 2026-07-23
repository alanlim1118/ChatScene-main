"""Scenario Description:

In a top-down view of a traffic scenario, a green ego vehicle is driving straight in a lane bounded by a dashed white line above and a solid yellow line below. A green arrow indicates the vehicle's forward motion towards a green rectangular object located directly ahead in the same lane. This object is described as a passable item, such as a manhole lid or a small branch, which the ego vehicle must react to while pursuing its objective of continuing straight along the road.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
OBJ_BLUEPRINT = 'static.prop.dirtdebris01'

param OBJ_DIST = VerifaiRange(20, 40)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

# Spawn object directly ahead in the same lane
objSpawnPt = new OrientedPoint at (egoSpawnPt offset by (globalParameters.OBJ_DIST, 0))

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL

obstacle = new Prop at objSpawnPt,
    with blueprint OBJ_BLUEPRINT

terminate when (distance to egoSpawnPt) > TERM_DIST
