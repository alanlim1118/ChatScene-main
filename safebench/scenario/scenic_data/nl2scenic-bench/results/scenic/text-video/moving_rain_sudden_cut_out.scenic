"""Scenario Description:

The ego vehicle travels forward on a wet, divided highway during rainy conditions with active windshield wipers clearing reduced visibility. A dark-colored SUV drives directly ahead in the same lane before abruptly illuminating its brake lights and decelerating sharply. In response to a stationary obstacle further ahead, the lead vehicle suddenly swerves into the right lane to avoid a collision. This unexpected maneuver forces the ego vehicle into an emergency braking and evasive swerving sequence, ultimately causing the ego vehicle to veer right and collide with the lead vehicle as it attempts to navigate the hazardous, low-traction environment.

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
LEAD_MODEL = "vehicle.tesla.modely"  # Dark SUV-like vehicle
OBSTACLE_MODEL = "static.prop.container"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = Range(7, 10)
param OPT_LEAD_DIST = Range(15, 25)           # Initial distance of lead vehicle ahead of ego
param OPT_OBSTACLE_DIST = Range(30, 45)       # Distance of obstacle ahead of lead vehicle
param OPT_BRAKE_TRIGGER_DIST = Range(12, 18)  # Distance at which lead vehicle brakes
param OPT_SWERVE_TRIGGER_DIST = Range(8, 14)  # Distance at which lead vehicle swerves right
param OPT_EGO_BRAKE_DIST = Range(10, 16)      # Distance at which ego initiates emergency brake

OPT_LEAD_BRAKE_AMOUNT = 1.0
OPT_EGO_BRAKE_AMOUNT = 1.0

#################################
# WEATHER                       #
#################################

param weather = WeatherConditions(
    precipitation=Range(0.6, 0.9),
    cloudiness=Range(0.7, 1.0),
    wetness=Range(0.8, 1.0),
    fogDensity=Range(0.1, 0.3)
)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LeadVehicleBehavior(lead_speed, brake_trigger_dist, swerve_trigger_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=lead_speed) until (distance from self to ObstacleAgent < brake_trigger_dist)
        take SetBrakeAction(brake_amount)
        do FollowLaneBehavior(target_speed=0) until (distance from self to ObstacleAgent < swerve_trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=lead_speed * 0.5)
        do FollowLaneBehavior(target_speed=lead_speed * 0.3)
    interrupt when (collision with ego):
        terminate

behavior EgoEmergencyBehavior(ego_speed, brake_dist, brake_amount):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to LeadAgent < brake_dist):
        take SetBrakeAction(brake_amount)
        do LaneChangeBehavior(laneSectionToSwitch=self.lane._laneToRight, target_speed=ego_speed * 0.4)
        do FollowLaneBehavior(target_speed=ego_speed * 0.2)
    terminate when (collision with LeadAgent) or (self.speed < 0.1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find highway lane sections that have a right neighbor (divided highway)
highwayLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            # Prefer longer straight sections typical of highways
            if laneSec.length > 50:
                highwayLaneSecs.append(laneSec)

require len(highwayLaneSecs) > 0

egoLaneSec = Uniform(*highwayLaneSecs)
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

obstacleOffset = globalParameters.OPT_LEAD_DIST + globalParameters.OPT_OBSTACLE_DIST
obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for obstacleOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Stationary obstacle in the left lane ---
ObstacleAgent = new Object at obstacleSpawnPt,
    with blueprint OBSTACLE_MODEL,
    with regionContainedIn egoLaneSec,
    with heading obstacleSpawnPt.heading

# --- Lead dark SUV ---
LeadAgent = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with regionContainedIn egoLaneSec,
    with color (0.05, 0.05, 0.05),  # Dark colored
    with behavior LeadVehicleBehavior(
        globalParameters.OPT_LEAD_SPEED,
        globalParameters.OPT_BRAKE_TRIGGER_DIST,
        globalParameters.OPT_SWERVE_TRIGGER_DIST,
        OPT_LEAD_BRAKE_AMOUNT
    )

# --- Ego vehicle ---
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn egoLaneSec,
    with windshieldWipers True,
    with behavior EgoEmergencyBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_BRAKE_DIST,
        OPT_EGO_BRAKE_AMOUNT
    )

# Ensure sufficient road length ahead for the scenario to play out
require distance from egoSpawnPt to intersection >= 80
require egoLaneSec.length >= 100

terminate when (collision between ego and LeadAgent) or (simulation.time > 30)