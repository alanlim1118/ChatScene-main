"""Scenario Description:

The ego vehicle initiates a driver-requested lane change on a multi-lane road, but must detect and yield to a high-speed vehicle approaching from the rear in the target lane, delaying the maneuver until the overtaking vehicle has safely passed.

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

param ADV_SPEED = Range(12, 15)      # High-speed overtaking vehicle
param ADV_SPAWN_DIST = Range(10, 20) # Spawn distance behind ego in the target lane

INIT_DIST = 60

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
targetLaneSec = egoLaneSec._laneToLeft

# Place the adversary in the target lane, behind the ego
targetLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from targetLanePt for -globalParameters.ADV_SPAWN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
