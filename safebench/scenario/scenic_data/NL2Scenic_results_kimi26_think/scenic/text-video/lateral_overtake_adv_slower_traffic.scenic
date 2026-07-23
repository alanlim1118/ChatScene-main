"""Scenario Description:

The ego vehicle travels along a curving, uphill rural highway bounded by concrete barriers, following a black Volkswagen sedan which is trailing a white sedan and a large red truck covered with a blue tarp. As the group proceeds, the ego vehicle initiates an overtaking maneuver by moving into the left lane to pass the black sedan, but at the exact same moment, the black sedan also steers left to overtake the slower white sedan and truck ahead. This simultaneous action places both cars in the oncoming lane side-by-side, creating a dangerous conflict just as a scooter rider wearing a yellow helmet approaches in the opposite direction. The black sedan's brake lights illuminate as it navigates the tight space and merges back into the lane directly in front of the ego vehicle, resulting in a near-miss or scraping side-swipe collision that forces the ego vehicle to abort the pass and resume following the black sedan.

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
BLACK_MODEL = "vehicle.seat.leon"          # Proxy for black Volkswagen sedan
WHITE_MODEL = "vehicle.audi.a2"            # White sedan
TRUCK_MODEL = "vehicle.carlamotors.carlacola"  # Large truck
SCOOTER_MODEL = "vehicle.yamaha.yzf"       # Motorcycle proxy for scooter

param OPT_EGO_SPEED = Range(12, 16)
param OPT_BLACK_SPEED = Range(10, 14)
param OPT_WHITE_SPEED = Range(8, 12)
param OPT_TRUCK_SPEED = Range(6, 10)
param OPT_SCOOTER_SPEED = Range(10, 14)

param OPT_BLACK_TRIGGER_DIST = Range(15, 25)   # Distance to white sedan to trigger overtake
param OPT_EGO_TRIGGER_DIST = Range(12, 20)     # Distance to black sedan to trigger overtake
param OPT_SCOOTER_OFFSET = Range(50, 80)       # Distance along oncoming lane from projection point

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior TruckBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior WhiteSedanBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior ScooterBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior BlackSedanBehavior(speed, trigger_dist, oncoming_lane, return_lane):
    # Follow the white sedan in the forward lane
    do FollowLaneBehavior(target_speed=speed) until (distance from self to WhiteSedan < trigger_dist)
    # Initiate overtake into the oncoming lane
    do LaneChangeBehavior(laneSectionToSwitch=oncoming_lane, target_speed=speed)
    # Continue until close to the approaching scooter, then brake and merge back
    do FollowLaneBehavior(target_speed=speed) until (distance from self to Scooter < 25)
    take SetBrakeAction(1)
    do LaneChangeBehavior(laneSectionToSwitch=return_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

behavior EgoBehavior(ego_speed, trigger_dist, oncoming_lane, return_lane):
    try:
        # Follow the black sedan
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to BlackSedan < trigger_dist)
        # Initiate overtake into the oncoming lane
        do LaneChangeBehavior(laneSectionToSwitch=oncoming_lane, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to BlackSedan < 5):
        # Black sedan merged back; near-miss conflict
        take SetBrakeAction(1)
        do WaitBehavior() for 2 seconds
        # Abort the pass and return to the original lane
        do LaneChangeBehavior(laneSectionToSwitch=return_lane, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward lane that has an oncoming (left) lane
twoLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            twoLaneSecs.append(laneSec)

egoLaneSec = Uniform(*twoLaneSecs)
oncomingLaneSec = egoLaneSec._laneToLeft

# Spawn points along the forward lane (front to back)
truckSpawnPt = new OrientedPoint in egoLaneSec.centerline
whiteSpawnPt = new OrientedPoint following roadDirection from truckSpawnPt for Range(20, 30)
blackSpawnPt = new OrientedPoint following roadDirection from whiteSpawnPt for Range(15, 25)
egoSpawnPt = new OrientedPoint following roadDirection from blackSpawnPt for Range(15, 25)

# Scooter spawn point in the oncoming lane, approaching the conflict zone
oncomingProj = oncomingLaneSec.centerline.project(blackSpawnPt.position)
scooterSpawnPt = new OrientedPoint following roadDirection from oncomingProj for globalParameters.OPT_SCOOTER_OFFSET

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Large red truck (slow, leading the pack) ---
Truck = new Car at truckSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior TruckBehavior(globalParameters.OPT_TRUCK_SPEED)

# --- White sedan (slow, behind truck) ---
WhiteSedan = new Car at whiteSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint WHITE_MODEL,
    with behavior WhiteSedanBehavior(globalParameters.OPT_WHITE_SPEED)

# --- Scooter in the oncoming lane (approaching) ---
Scooter = new Car at scooterSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with blueprint SCOOTER_MODEL,
    with behavior ScooterBehavior(globalParameters.OPT_SCOOTER_SPEED)

# --- Black sedan (behind white sedan, ahead of ego) ---
BlackSedan = new Car at blackSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint BLACK_MODEL,
    with behavior BlackSedanBehavior(
        globalParameters.OPT_BLACK_SPEED,
        globalParameters.OPT_BLACK_TRIGGER_DIST,
        oncomingLaneSec,
        egoLaneSec
    )

# --- Ego vehicle (following black sedan) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_TRIGGER_DIST,
        oncomingLaneSec,
        egoLaneSec
    )

require distance to intersection >= 100