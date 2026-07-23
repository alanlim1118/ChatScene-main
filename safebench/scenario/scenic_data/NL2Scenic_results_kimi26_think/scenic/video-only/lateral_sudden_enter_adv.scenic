"""Scenario Description:

The ego vehicle travels forward on a two-lane rural highway under clear daylight conditions, initially following a silver sedan. Suddenly, the silver sedan pulls out from the right shoulder area and cuts sharply across the ego vehicle's lane to attempt a left turn, directly obstructing the path. This unexpected maneuver forces the ego vehicle into a critical situation involving emergency braking and swerving, which results in a collision. The sequence concludes with the ego vehicle's windshield wipers activating and the car coming to a near stop near the grassy shoulder, indicating the aftermath of the crash.
"""

#################################
# MAP AND MODEL                 #
#################################
Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################
EGO_MODEL = "vehicle.lincoln.mkz_2017"
SEDAN_MODEL = "vehicle.toyota.prius"  # sedan blueprint

param OPT_EGO_SPEED = Range(12, 18)              # m/s (~43-65 km/h)
param OPT_SEDAN_TRIGGER_DIST = Range(20, 30)      # distance for sedan to start maneuver
param OPT_BRAKE_DIST = Range(10, 15)              # distance for ego to react
param OPT_SHOULDER_OFFSET = Range(3.5, 4.5)       # meters from lane center to shoulder

OPT_FULL_BRAKE = 1
OPT_SWERVE_STEER = -0.7   # negative = right, toward shoulder

#################################
# AGENT BEHAVIORS               #
#################################
behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to Sedan < brake_dist):
        # Emergency braking and swerving to the right shoulder
        take SetThrottleAction(0)
        take SetBrakeAction(OPT_FULL_BRAKE)
        take SetSteerAction(OPT_SWERVE_STEER)
        do WaitBehavior() for 2 seconds
        # Aftermath: straighten and come to near stop near shoulder
        take SetSteerAction(0.1)
        take SetBrakeAction(0.8)
        do WaitBehavior()
    terminate

behavior SedanBehavior(trigger_dist):
    # Remain stationary on the shoulder until ego approaches
    while (distance from self to ego) > trigger_dist:
        wait
    # Suddenly pull out and cut sharply left across the lane
    while True:
        take SetThrottleAction(0.9)
        take SetSteerAction(0.9)   # sharp left turn
        take SetBrakeAction(0)

#################################
# SPATIAL RELATIONS             #
#################################
# Select a forward lane section on the rural highway
forwardLaneSecs = [sec for lane in network.lanes for sec in lane.sections if sec.isForward]
egoLaneSec = Uniform(*forwardLaneSecs)

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Spawn sedan on the right shoulder, ahead of ego
sedanBasePt = new OrientedPoint following roadDirection from egoSpawnPt for Range(40, 60)
sedanSpawnPt = new OrientedPoint right of sedanBasePt by globalParameters.OPT_SHOULDER_OFFSET,
    facing roadDirection

#################################
# SCENARIO SPECIFICATION        #
#################################
# Ego vehicle traveling in the lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DIST
    )

# Silver sedan initially stopped on the right shoulder
Sedan = new Car at sedanSpawnPt,
    with heading sedanSpawnPt.heading,
    with regionContainedIn None,
    with blueprint SEDAN_MODEL,
    with behavior SedanBehavior(globalParameters.OPT_SEDAN_TRIGGER_DIST)

require distance to intersection >= 100

terminate after 20 seconds