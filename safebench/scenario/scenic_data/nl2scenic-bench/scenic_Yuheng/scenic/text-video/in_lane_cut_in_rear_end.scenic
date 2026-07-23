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
ADV_MODEL = "vehicle.tesla.model3"

# Speeds in m/s (118 km/h ~ 32.8 m/s, 100 km/h ~ 27.8 m/s, 10 km/h ~ 2.8 m/s)
param OPT_EGO_SPEED = Range(30, 34)           # ~108-122 km/h
param OPT_ADV_SPEED = Range(35, 40)           # Faster than ego for overtaking
param OPT_EGO_BRAKE_SPEED = Range(2, 4)       # Target speed after braking (~7-14 km/h)

# Distance parameters
param OPT_ADV_START_DIST = Range(30, 50)      # How far behind ego the adv starts (in left lane)
param OPT_CUTIN_TRIGGER_DIST = Range(5, 15)   # Distance ahead of ego when adv initiates cut-in
param OPT_EGO_BRAKE_DIST = Range(8, 18)       # Distance at which ego brakes after cut-in
param OPT_MIN_LANE_LENGTH = 200               # Minimum lane section length needed

OPT_EGO_BRAKE_AMOUNT = 1.0                    # Severe braking
OPT_ADV_CUTIN_SPEED = globalParameters.OPT_ADV_SPEED  # Maintain speed during cut-in

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to AdvAgent < brake_dist and AdvAgent is visible):
        take SetBrakeAction(brake_amount)
        do WaitBehavior() for 5 seconds
        terminate

behavior AdvCutInBehavior(adv_speed, cutin_trigger_dist, target_lane):
    # Overtake: follow left lane until alongside/ahead of ego
    do FollowLaneBehavior(target_speed=adv_speed) until (
        (distance from self to ego) < cutin_trigger_dist and 
        (relative position of ego from self).y > -5
    )
    # Execute cut-in into ego's lane
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=adv_speed)
    # Continue in ego's lane after cut-in
    do FollowLaneBehavior(target_speed=adv_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a forward left neighbor (for overtaking from left)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward and
            laneSec.length >= globalParameters.OPT_MIN_LANE_LENGTH
        ):
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in right lane (the lane being driven in)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversary spawns behind ego in the left lane
leftLaneRefPt = leftLaneSec.centerline.project(egoSpawnPt.position)
advStartPt = new OrientedPoint following roadDirection from leftLaneRefPt for -globalParameters.OPT_ADV_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (red car, high speed, brakes on cut-in) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (1.0, 0.0, 0.0),
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# --- Adversarial vehicle (white car, overtakes then cuts in) ---
AdvAgent = new Car at advStartPt,
    with heading leftLaneSec.centerline.headingAt(advStartPt.position),
    with regionContainedIn leftLaneSec,
    with blueprint ADV_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior AdvCutInBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_CUTIN_TRIGGER_DIST,
        egoLaneSec
    )

require distance to intersection >= 100
terminate when (distance from ego to egoSpawnPt) > 300