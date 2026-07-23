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

param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)
param SAFETY_DIST = VerifaiRange(10, 20)
param OBJ_DIST = VerifaiRange(20, 40)

TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random road and lane for the ego
road = Uniform(*network.roads)
lane = Uniform(*road.lanes)

# Spawn ego on the lane centerline
egoSpawnPt = new OrientedPoint in lane.centerline

# Spawn object directly ahead in the same lane
objSpawnPt = new OrientedPoint at (egoSpawnPt offset by (globalParameters.OBJ_DIST, 0))

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

obstacle = new Prop at objSpawnPt,
    with blueprint OBJ_BLUEPRINT

terminate when (distance to egoSpawnPt) > TERM_DIST