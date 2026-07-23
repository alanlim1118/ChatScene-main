"""Scenario Description:

The ego vehicle travels along a curving, uphill rural highway bounded by concrete barriers, following a black Volkswagen sedan which is trailing a white sedan and a large red truck covered with a blue tarp. As the group proceeds, the ego vehicle initiates an overtaking maneuver by moving into the left lane to pass the black sedan, but at the exact same moment, the black sedan also steers left to overtake the slower white sedan and truck ahead. This simultaneous action places both cars in the oncoming lane side-by-side, creating a dangerous conflict just as a scooter rider wearing a yellow helmet approaches in the opposite direction. The black sedan's brake lights illuminate as it navigates the tight space and merges back into the lane directly in front of the ego vehicle, resulting in a near-miss or scraping side-swipe collision that forces the ego vehicle to abort the pass and resume following the black sedan.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural highway map with curves and elevation changes
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
BLACK_SEDAN_MODEL = "vehicle.volkswagen.t2"
WHITE_SEDAN_MODEL = "vehicle.tesla.model3"
RED_TRUCK_MODEL = "vehicle.carlamotors.firetruck"  # Large truck; blue tarp approximated via color
SCOOTER_MODEL = "vehicle.diamondback.century"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BLACK_SEDAN_SPEED = Range(6, 9)
param OPT_WHITE_SEDAN_SPEED = Range(4, 6)
param OPT_TRUCK_SPEED = Range(3, 5)
param OPT_SCOOTER_SPEED = Range(5, 8)

param OPT_FOLLOW_DIST = Range(15, 25)        # Ego follows black sedan at this distance
param OPT_PLATOON_GAP = Range(10, 18)        # Gap between platoon vehicles
param OPT_OVERTAKE_TRIGGER_DIST = Range(30, 50)  # Distance at which ego initiates overtake
param OPT_MERGE_BACK_DIST = Range(8, 15)     # Distance after which black sedan merges back
param OPT_BRAKE_THRESHOLD = Range(5, 10)     # Near-miss / collision detection threshold

OPT_EGO_BRAKE_AMOUNT = 1.0
OPT_BLACK_BRAKE_AMOUNT = 0.8

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoOvertakeBehavior(ego_speed, follow_dist, overtake_trigger, brake_thresh):
    try:
        # Follow black sedan in right lane
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to BlackSedan < overtake_trigger)
        # Initiate overtake to left (oncoming) lane
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToLeft, target_speed=ego_speed)
        # Continue in oncoming lane
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to BlackSedan < brake_thresh):
        # Abort pass due to near-miss / side-swipe conflict
        take SetBrakeAction(OPT_EGO_BRAKE_AMOUNT)
        do WaitBehavior() for 2 seconds
        # Merge back to right lane behind black sedan
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    terminate

behavior BlackSedanOvertakeBehavior(black_speed, platoon_gap, merge_back_dist, brake_amount):
    # Follow white sedan initially
    do FollowLaneBehavior(target_speed=black_speed) until (distance from self to WhiteSedan > platoon_gap + 5)
    # Simultaneously steer left to overtake (same timing as ego)
    do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToLeft, target_speed=black_speed)
    # Briefly travel in oncoming lane alongside ego
    do FollowLaneBehavior(target_speed=black_speed) for Range(2, 4) seconds
    # Brake and merge back in front of ego
    take SetBrakeAction(brake_amount)
    do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=black_speed)
    do FollowLaneBehavior(target_speed=black_speed)

behavior SlowFollowBehavior(target_speed):
    while True:
        do FollowLaneBehavior(target_speed=target_speed)

behavior OncomingScooterBehavior(scooter_speed):
    while True:
        do FollowLaneBehavior(target_speed=scooter_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable curved, multi-lane road section without intersections
candidateSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and sec._laneToLeft.isForward:
            # Prefer sections with some curvature (non-zero average curvature proxy)
            if len(sec.centerline.points) > 10:
                candidateSections.append(sec)

require len(candidateSections) > 0
egoLaneSec = Uniform(*candidateSections)
oncomingLaneSec = egoLaneSec._laneToLeft

# Spawn ego in right lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Black sedan ahead of ego in same lane
blackSedanSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_FOLLOW_DIST

# White sedan ahead of black sedan
whiteSedanSpawnPt = new OrientedPoint following roadDirection from blackSedanSpawnPt for globalParameters.OPT_PLATOON_GAP

# Red truck ahead of white sedan
truckSpawnPt = new OrientedPoint following roadDirection from whiteSedanSpawnPt for globalParameters.OPT_PLATOON_GAP

# Scooter in oncoming lane, approaching from ahead
scooterAheadDist = Range(80, 120)
oncomingRefPt = oncomingLaneSec.centerline.project(blackSedanSpawnPt.position)
scooterSpawnPt = new OrientedPoint following roadDirection from oncomingRefPt for scooterAheadDist,
    with heading oncomingRefPt.heading + 180 deg  # Facing opposite direction

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoOvertakeBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_FOLLOW_DIST,
        globalParameters.OPT_OVERTAKE_TRIGGER_DIST,
        globalParameters.OPT_BRAKE_THRESHOLD
    )

# Black Volkswagen sedan (adversarial overtaker)
BlackSedan = new Car at blackSedanSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint BLACK_SEDAN_MODEL,
    with color (0, 0, 0),
    with behavior BlackSedanOvertakeBehavior(
        globalParameters.OPT_BLACK_SEDAN_SPEED,
        globalParameters.OPT_PLATOON_GAP,
        globalParameters.OPT_MERGE_BACK_DIST,
        OPT_BLACK_BRAKE_AMOUNT
    )

# White sedan (slow lead vehicle)
WhiteSedan = new Car at whiteSedanSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint WHITE_SEDAN_MODEL,
    with color (1, 1, 1),
    with behavior SlowFollowBehavior(globalParameters.OPT_WHITE_SEDAN_SPEED)

# Red truck with blue tarp (slowest lead vehicle)
RedTruck = new Car at truckSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint RED_TRUCK_MODEL,
    with color (0.8, 0.1, 0.1),
    with behavior SlowFollowBehavior(globalParameters.OPT_TRUCK_SPEED)

# Oncoming scooter rider with yellow helmet
ScooterRider = new Car at scooterSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with blueprint SCOOTER_MODEL,
    with color (1, 0.9, 0),  # Yellow to approximate helmet visibility
    with behavior OncomingScooterBehavior(globalParameters.OPT_SCOOTER_SPEED)

# Ensure spawn region is far from intersections for rural highway feel
require distance from egoSpawnPt to nearest intersection >= 100