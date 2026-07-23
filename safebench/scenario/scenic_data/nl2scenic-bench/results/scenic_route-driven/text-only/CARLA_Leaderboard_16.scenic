"""Scenario Description:

The ego-vehicle encounters a slow moving hazard blocking part of the lane. The ego-vehicle must brake or maneuver to avoid it next to a lane of traffic moving in the opposite direction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_HAZARD_SPEED = Range(1, 3)
param OPT_HAZARD_DIST = Range(40, 60)           # Distance ahead of ego where hazard is placed
param OPT_ONCOMING_DIST = Range(80, 120)        # Distance ahead of hazard where oncoming car starts
param OPT_ONCOMING_SPEED = Range(8, 12)

#################################
# AGENT BEHAVIORS               #
#################################

behavior HazardBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior OncomingBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

# Place hazard ahead of ego in same lane
hazardSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_HAZARD_DIST

# Place oncoming car ahead of hazard in the opposite lane
adjOppLaneSec = egoLaneSec._laneToLeft
oncomingRefPt = adjOppLaneSec.centerline.project(hazardSpawnPt.position)
OncomingSpawnPt = new OrientedPoint following roadDirection from oncomingRefPt for globalParameters.OPT_ONCOMING_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

# Slow-moving hazard blocking part of the lane
Hazard = new Car at hazardSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior HazardBehavior(globalParameters.OPT_HAZARD_SPEED)

# Oncoming traffic in opposite lane
OncomingCar = new Car at OncomingSpawnPt,
    with heading egoSpawnPt.heading + 180 deg,
    with regionContainedIn adjOppLaneSec,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED)

require distance to intersection > 100
