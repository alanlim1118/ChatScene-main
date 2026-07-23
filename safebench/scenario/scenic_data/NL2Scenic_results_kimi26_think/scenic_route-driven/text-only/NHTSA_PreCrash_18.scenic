"""Scenario Description:

Vehicle is changing lanes or passing in an urban area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph; and closes in on a lead vehicle.

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

# 55 mph ≈ 24.6 m/s
param EGO_SPEED = Range(20, 24.6)
param LEAD_SPEED = globalParameters.EGO_SPEED * Uniform(0.5, 0.75)
param LEAD_DISTANCE = Range(40, 60)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
spawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
initLaneSec = network.laneSectionAt(spawnPt)

leadSpawnPt = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawnPt,
    with regionContainedIn initLaneSec

lead = new Car at leadSpawnPt,
    with regionContainedIn initLaneSec,
    with behavior LeadBehavior()

require (distance from ego to intersection) > 100
require (distance from lead to intersection) > 100

terminate when (distance from ego to spawnPt) > 200
