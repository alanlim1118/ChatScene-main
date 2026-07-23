"""Scenario Description:

While traveling on a straight section, the ego vehicle wants to perform a lane change to overtake a lead vehicle but remains in its current lane because it detects a motorcycle filtering or approaching rapidly from behind in the adjacent lane, ensuring the target space is clear before moving.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MOTO_MODEL = "vehicle.kawasaki.ninja"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.4, 0.6)
param OPT_MOTO_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.3, 1.6)
param OPT_LEAD_DIST = Range(25, 35)
param OPT_MOTO_BEHIND_DIST = Range(30, 50)

#################################
# AGENT BEHAVIORS               #
#################################

# Lead vehicle drives slowly in ego's lane
behavior LeadBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

# Motorcycle approaches rapidly from behind in the adjacent (left) lane
behavior MotoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
targetLaneSec = egoLaneSec._laneToLeft

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

# Project ego position onto target lane, then place motorcycle behind that point
targetLaneProj = targetLaneSec.centerline.project(egoSpawnPt.position)
motoSpawnPt = new OrientedPoint following roadDirection from targetLaneProj for -globalParameters.OPT_MOTO_BEHIND_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

LeadVehicle = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior LeadBehavior(globalParameters.OPT_LEAD_SPEED)

MotoAgent = new Motorcycle at motoSpawnPt,
    with regionContainedIn targetLaneSec,
    with blueprint MOTO_MODEL,
    with behavior MotoBehavior(globalParameters.OPT_MOTO_SPEED)

require distance from ego to intersection >= 100
require distance from LeadVehicle to intersection >= 80
require distance from MotoAgent to intersection >= 80

terminate when distance from ego to egoSpawnPt > 150
