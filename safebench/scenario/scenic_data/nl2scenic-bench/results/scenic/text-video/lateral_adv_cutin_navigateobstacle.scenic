"""Scenario Description:

The ego vehicle proceeds straight on a multi-lane city road under bright, hazy sunlight that creates significant glare on the windshield, following a large box truck in the same lane. A black sedan suddenly enters the frame from the right, cutting sharply into the ego vehicle's lane in an attempt to bypass a stationary black vehicle stopped in the rightmost lane or shoulder. This abrupt maneuver causes the black sedan to cross directly into the ego vehicle's path, resulting in a side-impact collision. The black sedan comes to a halt in the lane ahead of the ego vehicle, while the stationary vehicle it was avoiding remains visible to the right.

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
TRUCK_MODEL = "vehicle.carlamotors.firetruck"  # Large box truck approximation
SEDAN_MODEL = "vehicle.tesla.model3"
STATIONARY_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)           # Ego cruising speed (m/s)
param OPT_TRUCK_SPEED = Range(6, 9)          # Truck speed (slower than ego)
param OPT_SEDAN_SPEED = Range(10, 14)        # Sedan speed when cutting in
param OPT_FOLLOW_DIST = Range(20, 30)        # Ego following distance behind truck
param OPT_STATIONARY_OFFSET = Range(30, 50)  # Distance ahead of ego where stationary car is placed
param OPT_CUT_IN_TRIGGER = Range(15, 25)     # Distance from stationary car when sedan begins cut-in
param OPT_SEDAN_LATERAL_SPEED = 3.0          # Lateral speed for sharp cut-in

# Weather parameters for bright hazy sunlight with glare
param weather = Weather(
    cloudiness=20,
    precipitation=0,
    precipitation_deposits=0,
    wind_intensity=10,
    sun_azimuth_angle=45,
    sun_altitude_angle=70,
    fog_density=30,
    fog_distance=50,
    wetness=0
)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowTruckBehavior(target_speed, follow_dist):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to LeadingTruck < follow_dist):
        do FollowLaneBehavior(target_speed=target_speed * 0.7)

behavior CutInAndStopBehavior(trigger_vehicle, lateral_speed, stop_distance):
    # Wait until close enough to the stationary vehicle to initiate cut-in
    while distance from self to trigger_vehicle > globalParameters.OPT_CUT_IN_TRIGGER:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_SEDAN_SPEED)
    
    # Sharp left lane change into ego's lane
    do LaneChangeBehavior(
        laneSectionToSwitch=self.lane._laneToLeft,
        target_speed=globalParameters.OPT_SEDAN_SPEED
    )
    
    # Continue briefly in new lane then stop
    do FollowLaneBehavior(target_speed=globalParameters.OPT_SEDAN_SPEED) for 2 seconds
    take SetThrottleAction(0), SetBrakeAction(1)
    do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Find a multi-lane forward road section with at least two lanes
multiLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToLeft is not None and sec._laneToLeft.isForward:
            multiLaneSections.append(sec)

require len(multiLaneSections) > 0
egoLaneSec = Uniform(*multiLaneSections)
rightLaneSec = egoLaneSec._laneToLeft  # In CARLA/Scenic, _laneToLeft is the right neighbor in forward direction

# Spawn ego on the left lane centerline
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place leading truck ahead of ego in same lane
truckSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_FOLLOW_DIST

# Place stationary vehicle in right lane ahead of truck
rightLaneRefPt = rightLaneSec.centerline.project(truckSpawnPt.position)
stationarySpawnPt = new OrientedPoint following roadDirection from rightLaneRefPt for globalParameters.OPT_STATIONARY_OFFSET

# Place adversarial sedan in right lane between truck and stationary vehicle
sedanStartOffset = globalParameters.OPT_STATIONARY_OFFSET * 0.5
sedanSpawnPt = new OrientedPoint following roadDirection from rightLaneRefPt for sedanStartOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle following the truck
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    with behavior EgoFollowTruckBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_FOLLOW_DIST
    )

# Leading box truck in ego's lane
LeadingTruck = new Car at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with regionContainedIn egoLaneSec,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

# Stationary black vehicle in right lane (obstacle being avoided)
StationaryCar = new Car at stationarySpawnPt,
    with blueprint STATIONARY_MODEL,
    with color "0,0,0",
    with regionContainedIn rightLaneSec,
    with behavior WaitBehavior()

# Adversarial black sedan that cuts in sharply
AdvSedan = new Car at sedanSpawnPt,
    with blueprint SEDAN_MODEL,
    with color "0,0,0",
    with regionContainedIn rightLaneSec,
    with behavior CutInAndStopBehavior(
        StationaryCar,
        globalParameters.OPT_SEDAN_LATERAL_SPEED,
        5
    )

# Ensure sufficient road length ahead for the scenario to play out
require distance to intersection >= 80

terminate after 30 seconds