"""Scenario Description:

The ego vehicle drives straight on a wet, multi-lane city street under overcast conditions, passing shops and trees on both sides. A white SUV is visible in the right adjacent lane, moving slightly faster than the ego vehicle. Suddenly, the white SUV swerves sharply to the left, cutting directly into the ego vehicle's lane to avoid another vehicle merging from its right. This abrupt lane change causes a sudden side-impact collision, forcing the ego vehicle to decelerate rapidly as the white SUV crosses its path and moves toward the center of the road.

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
SUV_MODEL = "vehicle.audi.etron"
MERGER_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(6, 9)
param OPT_SUV_SPEED = globalParameters.OPT_EGO_SPEED + Range(2, 4)
param OPT_MERGER_SPEED = globalParameters.OPT_EGO_SPEED - Range(0, 2)

param OPT_SUV_DIST = Range(10, 15)
param OPT_MERGER_DIST = Range(8, 12)
param OPT_SWERVE_DIST = Range(6, 10)
param OPT_EGO_BRAKE_DIST = Range(4, 8)

#################################
# WEATHER                       #
#################################

param weather = {
    'cloudiness': 90,
    'precipitation': 65,
    'precipitation_deposits': 85,
    'sun_altitude_angle': 25,
    'sun_azimuth_angle': 0,
    'wind_intensity': 10,
    'fog_density': 10
}

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (distance from self to SUVCar < brake_dist):
        take SetBrakeAction(1.0)
        do WaitBehavior()

behavior SUVBehavior(speed, target_lane, trigger_dist):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to MergerCar < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

behavior MergerBehavior(speed, target_lane, delay):
    do FollowLaneBehavior(target_speed=speed) for delay
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a forward lane with two lanes to its right (ego -> SUV -> merger)
laneSecsWithTwoRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and
            laneSec._laneToRight is not None and laneSec._laneToRight.isForward and
            laneSec._laneToRight._laneToRight is not None and laneSec._laneToRight._laneToRight.isForward):
            laneSecsWithTwoRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithTwoRight)
suvLaneSec = egoLaneSec._laneToRight
mergerLaneSec = suvLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# SUV in the right adjacent lane, slightly ahead of ego
suvProj = suvLaneSec.centerline.project(egoSpawnPt.position)
suvSpawnPt = new OrientedPoint following roadDirection from suvProj for globalParameters.OPT_SUV_DIST

# Merger in the far right lane, slightly ahead of the SUV
mergerProj = mergerLaneSec.centerline.project(suvSpawnPt.position)
mergerSpawnPt = new OrientedPoint following roadDirection from mergerProj for globalParameters.OPT_MERGER_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (drives straight in its lane) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST
    )

# --- White SUV (right adjacent lane, faster, swerves left) ---
SUVCar = new Car at suvSpawnPt,
    with regionContainedIn suvLaneSec,
    with blueprint SUV_MODEL,
    with color (1, 1, 1),
    with behavior SUVBehavior(
        globalParameters.OPT_SUV_SPEED,
        egoLaneSec,
        globalParameters.OPT_SWERVE_DIST
    )

# --- Merging vehicle (far right lane, merges left into SUV lane) ---
MergerCar = new Car at mergerSpawnPt,
    with regionContainedIn mergerLaneSec,
    with blueprint MERGER_MODEL,
    with behavior MergerBehavior(
        globalParameters.OPT_MERGER_SPEED,
        suvLaneSec,
        1 second
    )

require distance to intersection >= 80
terminate when (distance from ego to egoSpawnPt) > 120