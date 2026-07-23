"""Scenario Description:

The ego vehicle travels along a multi-lane highway in clear daylight, positioned in the left lane with tall residential buildings visible to the left and a large tanker truck occupying the lane to the right. Directly ahead, a white SUV proceeds in the same lane until it suddenly locks its brakes, likely in response to a congested bottleneck or slowing traffic ahead. This abrupt deceleration forces the ego vehicle into an emergency braking scenario, but it is unable to stop in time and rear-ends the white SUV. The collision brings the ego vehicle to a halt directly behind the white car, while other vehicles, such as a silver van and a black SUV, continue to pass in the adjacent right lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
WHITE_SUV_MODEL = "vehicle.tesla.modely"
TANKER_MODEL = "vehicle.bh.crossbike"  # Placeholder; use actual tanker if available
SILVER_VAN_MODEL = "vehicle.volkswagen.t2"
BLACK_SUV_MODEL = "vehicle.audi.etron"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SUV_SPEED = Range(7, 10)
param OPT_TANKER_SPEED = Range(6, 9)
param OPT_PASSING_SPEED = Range(9, 13)

param OPT_SUV_BRAKE_DIST = Range(15, 25)       # Distance from ego when SUV brakes
param OPT_EGO_BRAKE_DIST = Range(8, 14)        # Distance at which ego begins emergency brake
param OPT_INITIAL_GAP = Range(20, 35)          # Initial gap between ego and white SUV
param OPT_TANKER_OFFSET = Range(5, 15)         # Longitudinal offset of tanker relative to ego

OPT_EGO_BRAKE_AMOUNT = 1.0
OPT_SUV_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to WhiteSUV < brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(brake_amount)
        do WaitBehavior() for 10 seconds
        terminate

behavior SuvBrakeBehavior(suv_speed, brake_trigger_dist, brake_amount):
    do FollowLaneBehavior(target_speed=suv_speed) until (distance from self to ego < brake_trigger_dist)
    take SetThrottleAction(0)
    take SetBrakeAction(brake_amount)
    do WaitBehavior() for 10 seconds

behavior PassingBehavior(pass_speed):
    do FollowLaneBehavior(target_speed=pass_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a right neighbor (multi-lane highway)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

require len(laneSecsWithRightLane) > 0

egoLaneSec = Uniform(*laneSecsWithRightLane)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point in left lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# White SUV ahead of ego in same lane
suvSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_INITIAL_GAP

# Tanker in right lane, roughly alongside ego
rightLaneRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
tankerSpawnPt = new OrientedPoint following roadDirection from rightLaneRefPt for globalParameters.OPT_TANKER_OFFSET

# Passing vehicles in right lane, further ahead and behind tanker
vanSpawnPt = new OrientedPoint following roadDirection from tankerSpawnPt for Range(20, 35)
blackSuvSpawnPt = new OrientedPoint following roadDirection from vanSpawnPt for Range(15, 25)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (left lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# --- White SUV (left lane, ahead of ego, will brake suddenly) ---
WhiteSUV = new Car at suvSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint WHITE_SUV_MODEL,
    with color "255,255,255",
    with behavior SuvBrakeBehavior(
        globalParameters.OPT_SUV_SPEED,
        globalParameters.OPT_SUV_BRAKE_DIST,
        OPT_SUV_BRAKE_AMOUNT
    )

# --- Tanker truck (right lane, beside ego) ---
TankerTruck = new Car at tankerSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint TANKER_MODEL,
    with behavior PassingBehavior(globalParameters.OPT_TANKER_SPEED)

# --- Silver van (right lane, passing) ---
SilverVan = new Car at vanSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint SILVER_VAN_MODEL,
    with color "192,192,192",
    with behavior PassingBehavior(globalParameters.OPT_PASSING_SPEED)

# --- Black SUV (right lane, passing) ---
BlackSUV = new Car at blackSuvSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint BLACK_SUV_MODEL,
    with color "0,0,0",
    with behavior PassingBehavior(globalParameters.OPT_PASSING_SPEED)

# Ensure sufficient distance from intersections for highway-like behavior
require distance from egoSpawnPt to intersection >= 80
require always (ego.laneSection._laneToRight is not None)

# Terminate after collision or timeout
terminate when (distance from ego to WhiteSUV < 3) or (simulation().currentTime > 30)