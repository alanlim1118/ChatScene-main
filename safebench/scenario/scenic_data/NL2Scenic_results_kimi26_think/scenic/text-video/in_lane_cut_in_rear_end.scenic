"""Scenario Description:

The ego vehicle, depicted as a red car, travels along a lane at a high speed, initially maintaining around 118 km/h. A white vehicle in the adjacent left lane approaches from behind and overtakes the ego vehicle. As the white vehicle pulls alongside, it executes a cut-in maneuver, merging into the ego vehicle's lane directly ahead. In response to this encroachment, the ego vehicle engages in severe braking, rapidly decelerating from over 100 km/h to under 10 km/h to avoid a rear-end collision with the cutting-in vehicle, illustrating a critical rear-end conflict scenario.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.a2"

# Speeds in m/s (118 km/h ≈ 32.8 m/s)
param OPT_EGO_SPEED = Range(30, 33)
param OPT_ADV_SPEED = Range(35, 40)          # Faster, to overtake
param OPT_ADV_INIT_DIST = Range(20, 40)      # Distance behind ego in left lane
param OPT_CUT_IN_DIST = Range(6, 10)           # Distance to ego to trigger cut-in
param OPT_BRAKE_DIST = Range(5, 8)             # Distance at which ego initiates severe braking
param OPT_BRAKE_DURATION = Range(3, 5)         # Seconds of severe braking

OPT_BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior SevereBrakeBehavior(brake_amount):
    while True:
        take SetBrakeAction(brake_amount)

behavior EgoBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to AdvAgent < brake_dist):
        do SevereBrakeBehavior(brake_amount) for globalParameters.OPT_BRAKE_DURATION
        do FollowLaneBehavior(target_speed=2)   # Continue at ~7 km/h (< 10 km/h)

behavior AdvBehavior(adv_speed, cut_in_dist, target_lane):
    # Approach in left lane until close to ego (alongside / beginning cut-in)
    do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego < cut_in_dist)
    # Cut into ego's lane directly ahead
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=adv_speed)
    # Continue ahead (slight slowdown to force a rear-end conflict)
    do FollowLaneBehavior(target_speed=adv_speed - 3)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# White vehicle starts behind ego in the left lane
leftLaneBasePt = leftLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from leftLaneBasePt for -globalParameters.OPT_ADV_INIT_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (red car, selected lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (1, 0, 0),
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DIST,
        OPT_BRAKE_AMOUNT
    )

# --- Adversarial white vehicle (left lane, behind ego) ---
AdvAgent = new Car at AdvSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 1, 1),
    with behavior AdvBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_CUT_IN_DIST,
        egoLaneSec
    )

require distance to intersection >= 100
terminate when (distance from ego to AdvAgent) > 60