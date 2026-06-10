"""Scenario Description:

The ego vehicle follows a lead vehicle at a constant speed in a straight lane until the lead vehicle 
performs a sudden lateral evasive maneuver to change lanes, abruptly revealing a stationary passenger 
car centered in the original path and requiring the ego vehicle to detect the obstacle and perform 
emergency braking to avoid a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(10, 15)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED
param OPT_EGO_BRAKE_DIST = Range(15, 20)      # Distance for ego to detect and brake for the revealed object
param OPT_INITIAL_LEAD_DIST = Range(15, 20)   # Initial distance from ego to lead vehicle
param OPT_REVEAL_DIST = Range(12, 18)         # Distance at which lead vehicle swerves to reveal obstacle

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        take SetBrakeAction(1)
        wait

behavior LeadVehicleBehavior(speed, obstacle, target_lane_sec):
    try:
        # Drive normally until getting close to the stationary obstacle
        do FollowLaneBehavior(target_speed=speed) until (distance from self to obstacle < globalParameters.OPT_REVEAL_DIST)
        
        # Perform sudden lateral evasive maneuver (lane change)
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=speed)
        
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        # Emergency braking once the lead car reveals the obstacle
        while True:
            take SetBrakeAction(1)
            take SetThrottleAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that have an adjacent lane to the left for the lead car to swerve into
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

# Define positions
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_INITIAL_LEAD_DIST

# The stationary obstacle is further down the same lane
obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for Range(30, 40)

#################################
# SCENARIO SPECIFICATION        #
#################################

# The stationary passenger car revealed by the lead vehicle
Obstacle = new Car at obstacleSpawnPt,
    with behavior WaitBehavior(),
    with regionContainedIn egoLaneSec

# The lead vehicle performing the evasive maneuver
LeadAgent = new Car at leadSpawnPt,
    with behavior LeadVehicleBehavior(globalParameters.OPT_LEAD_SPEED, Obstacle, adjLaneSec),
    with regionContainedIn egoLaneSec

# The ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_BRAKE_DIST),
    with regionContainedIn egoLaneSec

# Ensure we are on a long straight section and not at an intersection immediately
require distance to intersection > 50
require (distance from ego to Obstacle) > 45

# Termination condition
terminate when distance from ego to Obstacle < 2 or (ego.speed < 0.1 and distance from ego to Obstacle < 15)