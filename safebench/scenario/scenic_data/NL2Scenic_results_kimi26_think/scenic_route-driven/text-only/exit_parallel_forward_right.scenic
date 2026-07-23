"""Scenario Description:

The ego car travels straight forward within its lane. Ahead of it, an adversarial object exits parallel forward, leaving the ego-traffic area to the right.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_ADV_SPEED = Range(3, 7)
param OPT_ADV_START_DIST = Range(20, 40)   # Distance ahead of ego where adversarial starts
param OPT_ADV_EXIT_DIST = Range(5, 10)     # Distance after which adversarial changes lane to the right

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversarialBehavior(rightLane, startPt):
    # Drive forward briefly then exit to the right lane, leaving the ego-traffic area
    do FollowLaneBehavior(globalParameters.OPT_ADV_SPEED) until distance from self to startPt > globalParameters.OPT_ADV_EXIT_DIST
    do LaneChangeBehavior(laneSectionToSwitch=rightLane, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

rightLaneSec = egoLaneSec._laneToRight

# Place adversarial ahead in the same lane as the ego
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdversarialBehavior(rightLaneSec, advSpawnPt)

require distance to intersection >= 100  # Ensure the ego vehicle is far from any intersection
terminate when distance from ego to AdvAgent > 60
