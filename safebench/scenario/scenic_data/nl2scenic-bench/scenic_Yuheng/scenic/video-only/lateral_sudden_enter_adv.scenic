"""Scenario Description:

The ego vehicle travels forward on a two-lane rural highway under clear daylight conditions, initially following a silver sedan. Suddenly, the silver sedan pulls out from the right shoulder area and cuts sharply across the ego vehicle's lane to attempt a left turn, directly obstructing the path. This unexpected maneuver forces the ego vehicle into a critical situation involving emergency braking and swerving, which results in a collision. The sequence concludes with the ego vehicle's windshield wipers activating and the car coming to a near stop near the grassy shoulder, indicating the aftermath of the crash.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town06'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town06'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
SILVER_SEDAN_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)           # Ego cruising speed on rural highway
param OPT_LEAD_SPEED = Range(6, 9)           # Silver sedan initial speed (slower than ego)
param OPT_CUT_IN_TRIGGER_DIST = Range(25, 40) # Distance at which silver sedan begins cut-in
param OPT_BRAKE_TRIGGER_DIST = Range(10, 18)  # Distance at which ego initiates emergency brake
param OPT_SHOULDER_OFFSET = Range(2.5, 4.0)   # Lateral offset of silver sedan on right shoulder

OPT_EGO_BRAKE_AMOUNT = 1.0                   # Full emergency braking
OPT_POST_CRASH_WAIT = 5                      # Seconds to wait after collision for aftermath

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior SilverSedanCutInBehavior(initial_speed, trigger_dist, target_lane):
    """Silver sedan drives on shoulder then cuts sharply left across ego's lane."""
    try:
        do FollowLaneBehavior(target_speed=initial_speed) until (distance from self to ego < trigger_dist)
        # Cut sharply left across ego's lane toward left side (simulating left turn attempt)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=initial_speed * 0.7)
        # After crossing, decelerate as if attempting a left turn
        take SetThrottleAction(0.1), SetBrakeAction(0.5)
        do WaitBehavior() for 3 seconds
    interrupt when (collision with ego):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for OPT_POST_CRASH_WAIT
        terminate

behavior EgoEmergencyResponseBehavior(ego_speed, brake_trigger_dist, brake_amount):
    """Ego follows lane, then performs emergency braking and swerve when silver sedan cuts in."""
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to SilverSedan < brake_trigger_dist):
        # Emergency braking
        take SetBrakeAction(brake_amount), SetThrottleAction(0)
        # Attempt swerve to right (toward shoulder) to avoid collision
        take SetSteerAction(0.6)
        do WaitBehavior() for 1 seconds
        take SetSteerAction(0)
        # Aftermath: activate wipers and come to near stop
        take SetWindshieldWipersAction(True)
        take SetBrakeAction(0.8)
        do WaitBehavior() for OPT_POST_CRASH_WAIT
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable two-lane rural highway section with a right shoulder
laneSecsWithShoulder = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec.rightEdge is not None):
            laneSecsWithShoulder.append(laneSec)

require len(laneSecsWithShoulder) > 0

egoLaneSec = Uniform(*laneSecsWithShoulder)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in the right lane (driving lane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Silver sedan starts on the right shoulder, ahead of ego
shoulderPt = new OrientedPoint on egoLaneSec.rightEdge,
    ahead of egoSpawnPt by Range(30, 50)
silverSedanSpawnPt = new OrientedPoint left of shoulderPt by globalParameters.OPT_SHOULDER_OFFSET,
    with heading egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Clear daylight weather
param weather = Weather(precipitation=0, cloudiness=0, sunAltitude=70)

# Ego vehicle in driving lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyResponseBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_TRIGGER_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# Silver sedan starting on right shoulder
SilverSedan = new Car at silverSedanSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint SILVER_SEDAN_MODEL,
    with color (0.75, 0.75, 0.78),
    with behavior SilverSedanCutInBehavior(
        globalParameters.OPT_LEAD_SPEED,
        globalParameters.OPT_CUT_IN_TRIGGER_DIST,
        leftLaneSec
    )

# Ensure sufficient road ahead for the scenario to play out
require distance to intersection >= 120
require length of egoLaneSec.centerline >= 150

terminate after 30 seconds