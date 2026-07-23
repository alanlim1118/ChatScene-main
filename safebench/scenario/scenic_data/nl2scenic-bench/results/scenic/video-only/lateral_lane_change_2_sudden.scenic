"""Scenario Description:

On a bright, sunny day along a wide multi-lane city road flanked by tall apartment buildings, the ego vehicle proceeds forward in the middle lane. A green and yellow taxi, initially traveling in the lane to the right, suddenly executes a sharp and aggressive lane change, cutting across the lane markings directly into the ego vehicle's path. This abrupt maneuver places the taxi immediately in front of the ego vehicle, necessitating heavy braking to prevent a rear-end collision. The taxi stabilizes in the lane ahead, and the ego vehicle maintains a safe following distance as traffic flow resumes normalcy.

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
TAXI_MODEL = "vehicle.ford.mustang"  # Using a sedan as taxi base; color set via blueprint or attribute

param OPT_EGO_SPEED = Range(8, 12)        # Ego cruising speed (m/s)
param OPT_TAXI_SPEED = Range(9, 13)       # Taxi slightly faster before cut-in
param OPT_CUTIN_TRIGGER_DIST = Range(25, 40)  # Distance at which taxi initiates lane change
param OPT_BRAKE_DISTANCE = Range(8, 15)   # Distance threshold for ego to brake after cut-in
param OPT_FOLLOWING_DIST = Range(15, 25)  # Safe following distance after stabilization

OPT_EGO_BRAKE_AMOUNT = 1.0                # Full emergency brake
OPT_TAXI_LANE_CHANGE_SPEED = Range(6, 9)  # Speed during aggressive lane change

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(ego_speed, brake_distance, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to TaxiAgent < brake_distance and relative heading to TaxiAgent < 30 deg):
        take SetThrottleAction(0), SetBrakeAction(brake_amount)
        do WaitBehavior() for 2 seconds
        # Resume driving with safe following distance
        do FollowLaneBehavior(target_speed=ego_speed, min_gap=globalParameters.OPT_FOLLOWING_DIST)
    terminate after 30 seconds

behavior TaxiCutInBehavior(taxi_speed, cutin_trigger_dist, lane_change_speed, target_lane):
    # Drive normally in right lane until trigger distance
    do FollowLaneBehavior(target_speed=taxi_speed) until (distance from self to ego < cutin_trigger_dist)
    # Execute aggressive lane change into ego's lane
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=lane_change_speed)
    # Stabilize and continue in new lane
    do FollowLaneBehavior(target_speed=taxi_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right neighbor (middle lane of multi-lane road)
middleLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            middleLaneSections.append(laneSec)

require len(middleLaneSections) > 0

egoLaneSec = Uniform(*middleLaneSections)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight
rightLaneProjPt = rightLaneSec.centerline.project(egoSpawnPt.position)

# Place taxi ahead in the right lane so it can cut in
taxiOffset = Range(10, 20)
taxiSpawnPt = new OrientedPoint following roadDirection from rightLaneProjPt for taxiOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: bright sunny day
param weather = Weather(sun_altitude=80 deg, cloudiness=0.1, fog_density=0.0)

# Ego vehicle in middle lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_BRAKE_DISTANCE,
        OPT_EGO_BRAKE_AMOUNT
    )

# Taxi in right lane, green and yellow coloring
TaxiAgent = new Car at taxiSpawnPt,
    with regionContainedIn rightLaneSec,
    with heading egoSpawnPt.heading,
    with blueprint TAXI_MODEL,
    with color (0.2, 0.8, 0.1),  # Green-yellow tint
    with behavior TaxiCutInBehavior(
        globalParameters.OPT_TAXI_SPEED,
        globalParameters.OPT_CUTIN_TRIGGER_DIST,
        OPT_TAXI_LANE_CHANGE_SPEED,
        egoLaneSec
    )

# Ensure sufficient road length for the scenario
require distance to intersection >= 80

terminate after 30 seconds