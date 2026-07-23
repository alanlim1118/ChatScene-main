"""Scenario Description:

The ego vehicle drives forward on a multi-lane highway under clear late-afternoon conditions with snow visible on the road shoulders. A white sedan travels ahead in the left lane while a truck is visible further ahead in the right lane. A black sedan rapidly approaches from the left rear and aggressively cuts into the ego vehicle's lane. The black sedan immediately attempts to merge further right but hesitates and brakes hard as it encounters a truck in the adjacent lane. This sudden intrusion forces the ego vehicle into an emergency braking scenario to avoid a collision with the cutting vehicle, while the black sedan's erratic maneuvering and braking result in a collision with the truck.

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
WHITE_SEDAN_MODEL = "vehicle.audi.a2"
BLACK_SEDAN_MODEL = "vehicle.audi.tt"
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_WHITE_SEDAN_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(18, 22)
param OPT_TRUCK_SPEED = Range(6, 10)

param OPT_CUT_IN_DIST = Range(15, 25)          # Distance behind ego where black sedan starts
param OPT_WHITE_SEDAN_AHEAD = Range(40, 60)   # White sedan ahead in left lane
param OPT_TRUCK_AHEAD = Range(25, 35)          # Truck ahead in right lane
param OPT_BRAKE_DIST = Range(8, 12)            # Distance for ego to brake after cut-in
param OPT_TRUCK_TRIGGER_DIST = Range(15, 20)   # Distance at which black sedan brakes near truck

OPT_EGO_BRAKE = 1
OPT_ADV_BRAKE = 1

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to BlackSedan < brake_dist):
        take SetBrakeAction(brake_amount)
        do WaitBehavior()

behavior BlackSedanBehavior(adv_speed, ego_lane, right_lane, cut_in_dist, truck_dist, brake_amount):
    # Approach rapidly from left rear
    do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego < cut_in_dist)
    # Aggressively cut into ego's lane
    do LaneChangeBehavior(laneSectionToSwitch=ego_lane, target_speed=adv_speed)
    # Immediately attempt to merge further right; brake hard when encountering the truck
    try:
        do LaneChangeBehavior(laneSectionToSwitch=right_lane, target_speed=adv_speed)
    interrupt when (distance from self to Truck < truck_dist):
        take SetBrakeAction(brake_amount)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and
            laneSec._laneToRight is not None and laneSec._laneToRight.isForward):
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

# White sedan ahead in left lane
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
whiteSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_WHITE_SEDAN_AHEAD

# Truck ahead in right lane
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_TRUCK_AHEAD

# Black sedan behind in left lane
blackSpawnPt = new OrientedPoint following roadDirection from leftLanePt for -globalParameters.OPT_CUT_IN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (middle lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DIST,
        OPT_EGO_BRAKE
    )

# --- White sedan (left lane, ahead) ---
WhiteSedan = new Car at whiteSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint WHITE_SEDAN_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_WHITE_SEDAN_SPEED)

# --- Truck (right lane, ahead) ---
Truck = new Car at truckSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

# --- Black sedan (left lane, behind, aggressive cut-in) ---
BlackSedan = new Car at blackSpawnPt,
    with regionContainedIn leftLaneSec,
    with blueprint BLACK_SEDAN_MODEL,
    with behavior BlackSedanBehavior(
        globalParameters.OPT_ADV_SPEED,
        egoLaneSec,
        rightLaneSec,
        globalParameters.OPT_CUT_IN_DIST,
        globalParameters.OPT_TRUCK_TRIGGER_DIST,
        OPT_ADV_BRAKE
    )

require distance to intersection >= 100