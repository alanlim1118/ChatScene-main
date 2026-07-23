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
WHITE_SEDAN_MODEL = "vehicle.tesla.model3"
BLACK_SEDAN_MODEL = "vehicle.audi.a2"
TRUCK_MODEL = "vehicle.carlamotors.firetruck"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_WHITE_SEDAN_SPEED = globalParameters.OPT_EGO_SPEED - Range(1, 3)
param OPT_TRUCK_SPEED = globalParameters.OPT_EGO_SPEED - Range(3, 5)
param OPT_BLACK_SEDAN_SPEED = globalParameters.OPT_EGO_SPEED + Range(3, 6)

param OPT_WHITE_SEDAN_DIST = Range(25, 40)       # White sedan ahead of ego in same lane
param OPT_TRUCK_DIST = Range(40, 60)             # Truck ahead of ego in right lane
param OPT_BLACK_SEDAN_REAR_DIST = Range(15, 25)  # Black sedan behind ego in left-rear
param OPT_CUT_TRIGGER_DIST = Range(10, 18)       # Distance at which black sedan cuts in
param OPT_EGO_BRAKE_DIST = Range(8, 14)          # Distance at which ego brakes for cut-in
param OPT_BLACK_BRAKE_DIST_TO_TRUCK = Range(6, 12)  # Distance at which black sedan brakes for truck

OPT_EGO_BRAKE_AMOUNT = 1.0
OPT_BLACK_SEDAN_BRAKE_AMOUNT = 1.0

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
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate

behavior BlackSedanBehavior(cut_speed, cut_trigger_dist, brake_dist_to_truck, brake_amount):
    # Approach from behind at high speed
    do FollowLaneBehavior(target_speed=cut_speed) until (distance from self to ego < cut_trigger_dist)
    # Aggressively cut into ego's lane (right lane change)
    do LaneChangeBehavior(laneSectionToSwitch=ego.laneSection._laneToRight, target_speed=cut_speed)
    # Attempt to merge further right but encounter truck, brake hard
    try:
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=cut_speed)
    interrupt when (distance from self to Truck < brake_dist_to_truck):
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate

behavior WhiteSedanBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior TruckBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find highway-like lane sections with at least two adjacent forward lanes
multiLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward
            and laneSec._laneToRight._laneToRight is not None
            and laneSec._laneToRight._laneToRight.isForward):
            multiLaneSections.append(laneSec)

require len(multiLaneSections) > 0

egoLaneSec = Uniform(*multiLaneSections)
rightLaneSec = egoLaneSec._laneToRight
farRightLaneSec = rightLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

whiteSedanSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_WHITE_SEDAN_DIST

rightLaneRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following roadDirection from rightLaneRefPt for globalParameters.OPT_TRUCK_DIST

blackSedanSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for -globalParameters.OPT_BLACK_SEDAN_REAR_DIST

#################################
# WEATHER AND TIME              #
#################################

param weather = Weather(
    cloudiness=10,
    precipitation=0,
    wetness=20,
    fogDensity=0,
    sunAltitudeAngle=25,      # Late afternoon low sun
    sunAzimuthAngle=200
)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (middle-left lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# --- White sedan (ahead in same lane as ego) ---
WhiteSedan = new Car at whiteSedanSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint WHITE_SEDAN_MODEL,
    with behavior WhiteSedanBehavior(globalParameters.OPT_WHITE_SEDAN_SPEED)

# --- Truck (ahead in right lane) ---
Truck = new Car at truckSpawnPt,
    with regionContainedIn rightLaneSec,
    with blueprint TRUCK_MODEL,
    with behavior TruckBehavior(globalParameters.OPT_TRUCK_SPEED)

# --- Black sedan (approaching from left rear, will cut in then brake) ---
BlackSedan = new Car at blackSedanSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint BLACK_SEDAN_MODEL,
    with behavior BlackSedanBehavior(
        globalParameters.OPT_BLACK_SEDAN_SPEED,
        globalParameters.OPT_CUT_TRIGGER_DIST,
        globalParameters.OPT_BLACK_BRAKE_DIST_TO_TRUCK,
        OPT_BLACK_SEDAN_BRAKE_AMOUNT
    )

# Ensure sufficient road length ahead for the scenario to play out
require distance from egoSpawnPt to end of egoLaneSec >= 80

terminate after 30 seconds