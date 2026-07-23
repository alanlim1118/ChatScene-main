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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_SEDAN_SPEED = Range(8, 12)
param OPT_TRUCK_SPEED = Range(0, 2)

param OPT_BUS_DIST = Range(45, 55)
param OPT_TRUCK_DIST = Range(45, 55)
param OPT_SEDAN_DIST = Range(15, 25)

param OPT_LANE_CHANGE_TRIGGER = Range(15, 25)
param OPT_BRAKE_DIST = Range(5, 10)

OPT_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(target_speed, bus, sedan, lane_change_trigger, brake_dist, target_lane):
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to bus < lane_change_trigger)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to sedan < brake_dist):
        take SetBrakeAction(OPT_BRAKE_AMOUNT)
        take SetSteerAction(0.8)
        do WaitBehavior() for 3 seconds
        terminate

behavior SedanBehavior(target_speed, ego, trigger_dist, target_lane):
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior StationaryBehavior():
    while True:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior SlowBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward lane section that has a valid adjacent left lane
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leftLaneSec = egoLaneSec._laneToLeft

# Bus ahead in the ego lane (stationary / slowing)
busSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BUS_DIST

# Yellow box truck ahead in the left lane (slow)
truckSpawnPt = new OrientedPoint following roadDirection from leftLaneSec.centerline.project(egoSpawnPt.position) for globalParameters.OPT_TRUCK_DIST

# White sedan in the left lane, behind the truck but near the ego
sedanSpawnPt = new OrientedPoint following roadDirection from leftLaneSec.centerline.project(egoSpawnPt.position) for globalParameters.OPT_SEDAN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Blue and yellow bus (ahead in ego lane, stationary) ---
BusAgent = new Car at busSpawnPt,
    with regionContainedIn egoLaneSec,
    with color Color(0.1, 0.3, 0.8),
    with behavior StationaryBehavior()

# --- Yellow box truck (ahead in left lane, slow) ---
TruckAgent = new Car at truckSpawnPt,
    with regionContainedIn leftLaneSec,
    with color Color(0.9, 0.9, 0.1),
    with behavior SlowBehavior(globalParameters.OPT_TRUCK_SPEED)

# --- White sedan (in left lane, cuts right into ego's path) ---
SedanAgent = new Car at sedanSpawnPt,
    with regionContainedIn leftLaneSec,
    with color Color(1.0, 1.0, 1.0),
    with behavior SedanBehavior(
        globalParameters.OPT_SEDAN_SPEED,
        ego,
        globalParameters.OPT_LANE_CHANGE_TRIGGER,
        egoLaneSec
    )

# --- Ego vehicle ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        BusAgent,
        SedanAgent,
        globalParameters.OPT_LANE_CHANGE_TRIGGER,
        OPT_BRAKE_DIST,
        leftLaneSec
    )

require distance to intersection >= 80