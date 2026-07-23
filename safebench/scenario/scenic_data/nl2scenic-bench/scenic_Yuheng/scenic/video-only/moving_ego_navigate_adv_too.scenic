"""Scenario Description:

The ego vehicle travels forward on a sunny urban roadway, approaching a pedestrian overpass with a yellow box truck visible in the left lane and a large blue and yellow bus directly ahead in the current lane. As the ego vehicle closes the distance to the bus, which appears to be slowing down or stationary, the driver attempts to navigate around it by initiating a lane change to the left. However, a white sedan in the adjacent left lane simultaneously cuts into the exact space the ego vehicle is targeting. This conflict forces the ego vehicle into a sudden emergency braking and swerving maneuver to avoid the encroaching white car, resulting in a side-swipe collision as the vehicles converge while trying to bypass the bus.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
BUS_MODEL = "vehicle.volkswagen.t2"
TRUCK_MODEL = "vehicle.mercedes.sprinter"
SEDAN_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BUS_SPEED = Range(0, 2)            # Bus is slowing/stationary
param OPT_TRUCK_SPEED = Range(6, 9)          # Truck moving normally in left lane
param OPT_SEDAN_SPEED = Range(8, 12)         # Sedan matches ego speed initially

param OPT_BUS_DIST = Range(30, 45)           # Distance of bus ahead of ego
param OPT_TRUCK_DIST = Range(25, 40)         # Distance of truck ahead in left lane
param OPT_SEDAN_LATERAL_OFFSET = Range(30, 50)  # How far back sedan starts behind ego in left lane

param OPT_LANE_CHANGE_TRIGGER_DIST = Range(18, 25)  # When ego initiates lane change
param OPT_SEDAN_CUT_IN_TRIGGER_DIST = Range(12, 18) # When sedan cuts in
param OPT_EGO_BRAKE_DIST = Range(6, 10)             # Emergency brake threshold

OPT_EGO_BRAKE_AMOUNT = 1.0
OPT_SEDAN_CUT_IN_DURATION = 2.0  # seconds for cut-in maneuver

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, lane_change_trigger_dist, brake_dist, target_lane):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to BusAgent < lane_change_trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to SedanAgent < brake_dist):
        take SetBrakeAction(OPT_EGO_BRAKE_AMOUNT), SetSteerAction(-0.3)  # Emergency brake + swerve
        do WaitBehavior() for 5 seconds
        terminate

behavior SedanCutInBehavior(sedan_speed, cut_in_trigger_dist, target_lane):
    do FollowLaneBehavior(target_speed=sedan_speed) until (distance from self to ego < cut_in_trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=sedan_speed)
    do FollowLaneBehavior(target_speed=sedan_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a left neighbor (for left lane change)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leftLaneSec = egoLaneSec._laneToLeft

# Bus spawn point: directly ahead in ego's lane
busSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BUS_DIST

# Truck spawn point: ahead in left lane
leftLaneRefPt = leftLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from leftLaneRefPt for globalParameters.OPT_TRUCK_DIST

# Sedan spawn point: behind ego in left lane (will cut in)
sedanSpawnPt = new OrientedPoint following roadDirection from leftLaneRefPt for -globalParameters.OPT_SEDAN_LATERAL_OFFSET

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST,
        globalParameters.OPT_EGO_BRAKE_DIST,
        leftLaneSec
    )

# --- Bus (ahead in ego lane, slow/stationary) ---
BusAgent = new Car at busSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint BUS_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED)

# --- Yellow box truck (in left lane, ahead) ---
TruckAgent = new Car at truckSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

# --- White sedan (in left lane, cuts into ego's target space) ---
SedanAgent = new Car at sedanSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint SEDAN_MODEL,
    with behavior SedanCutInBehavior(
        globalParameters.OPT_SEDAN_SPEED,
        globalParameters.OPT_SEDAN_CUT_IN_TRIGGER_DIST,
        egoLaneSec  # Cuts INTO ego's original lane (the space ego is vacating/targeting)
    )

# Ensure sufficient distance from intersections for clean scenario execution
require distance to intersection >= 80

# Weather: sunny
param weather = 'Sunny'

terminate after 30 seconds