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

param OPT_PARKING_OFFSET = Range(2.0, 3.0)      # Lateral offset from lane center (m)
param OPT_ADV_START_DIST = Range(20, 40)        # Distance behind ego for traffic (m)
param OPT_EGO_SPEED = Range(3, 6)               # Ego speed after pull-out (m/s)
param OPT_ADV_SPEED = Range(8, 12)              # Traffic speed (m/s)
param OPT_EGO_WAIT = Range(2, 5)                # Seconds before ego pulls out
param OPT_SAFE_DISTANCE = Range(10, 15)         # Trigger distance for traffic braking (m)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Remain stationary in the parking bay for a random duration
    do FollowLaneBehavior(target_speed=0) for globalParameters.OPT_EGO_WAIT seconds
    # Pull out into the flow of traffic
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior TrafficBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when (distance from self to ego < globalParameters.OPT_SAFE_DISTANCE):
        take SetBrakeAction(1)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED * 0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane section away from intersections
egoLaneSec = Uniform(*[sec for lane in network.lanes for sec in lane.sections])
roadPt = new OrientedPoint in egoLaneSec.centerline

# Ego is parked parallel to the road, offset to the right (curb side)
egoSpawnPt = new OrientedPoint right of roadPt by globalParameters.OPT_PARKING_OFFSET,
    with heading roadPt.heading

# Traffic vehicle approaches from behind in the same lane
advSpawnPtRaw = new OrientedPoint following roadDirection from roadPt for -globalParameters.OPT_ADV_START_DIST
advSpawnPt = egoLaneSec.centerline.project(advSpawnPtRaw.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle parked at the side of the road
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Traffic vehicle in the same lane approaching from behind
AdvAgent = new Car at advSpawnPt,
    with heading roadPt.heading,
    with regionContainedIn egoLaneSec,
    with behavior TrafficBehavior()

require distance to intersection > 100