"""Scenario Description:

The ego-vehicle must exit a parallel parking bay into a flow of traffic.

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

param OPT_ADV_START_DIST = Range(20, 40)        # Distance behind ego for traffic (m)
param OPT_ADV_SPEED = Range(8, 12)              # Traffic speed (m/s)
param OPT_SAFE_DISTANCE = Range(10, 15)         # Trigger distance for traffic braking (m)

#################################
# AGENT BEHAVIORS               #
#################################

behavior TrafficBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when (distance from self to ego < globalParameters.OPT_SAFE_DISTANCE):
        take SetBrakeAction(1)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED * 0.5)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

# Point on the lane centerline aligned with the ego's parked (offset) position
roadPt = new OrientedPoint at egoLaneSec.centerline.project(egoSpawnPt.position),
    with heading egoSpawnPt.heading

# Traffic vehicle approaches from behind in the same lane
advSpawnPtRaw = new OrientedPoint following roadDirection from roadPt for -globalParameters.OPT_ADV_START_DIST
advSpawnPt = egoLaneSec.centerline.project(advSpawnPtRaw.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle parked at the side of the road
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

# Traffic vehicle in the same lane approaching from behind
AdvAgent = new Car at advSpawnPt,
    with heading roadPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior TrafficBehavior()

require distance to intersection > 100
