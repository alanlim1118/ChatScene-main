"""Scenario Description:
The ego vehicle follows a lead vehicle at a constant speed in the center of the lane until 
the lead vehicle suddenly performs an evasive swerve to an adjacent lane, 
revealing a stationary passenger car target directly in the ego vehicle's path. 
The ego vehicle should detect the obstacle and brake to a full stop without collision.
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
LEAD_MODEL = "vehicle.audi.a2"
OBSTACLE_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(10, 12)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED
param OPT_EGO_FOLLOW_DIST = Range(15, 20)
param OPT_OBSTACLE_DIST = Range(45, 55)

# Trigger distances
SWERVE_TRIGGER_DIST = 12   # Lead vehicle swerves when this close to obstacle
BRAKE_TRIGGER_DIST = 15    # Ego vehicle brakes when this close to obstacle

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_speed, swerve_lane, obstacle_obj):
    # Drive normally until close to the obstacle
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to obstacle_obj < SWERVE_TRIGGER_DIST)
    
    # Perform evasive swerve
    do LaneChangeBehavior(laneSectionToSwitchTo=swerve_lane, target_speed=target_speed)
    
    # Continue in new lane
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoVehicleBehavior(target_speed, obstacle_obj):
    try:
        # Follow the lane/lead vehicle
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to obstacle_obj < BRAKE_TRIGGER_DIST):
        # Emergency braking to a full stop
        while self.speed > 0.1:
            take SetThrottleAction(0), SetBrakeAction(1.0)
        
        # Hold brake at stop
        while True:
            take SetBrakeAction(1.0)
            if self.speed < 0.1:
                terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane section that has at least one adjacent lane to swerve into
laneSecsWithAdj = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Check for multi-lane road in the same direction
        if laneSec.isForward:
            if laneSec._laneToRight and laneSec._laneToRight.isForward:
                laneSecsWithAdj.append((laneSec, laneSec._laneToRight))
            elif laneSec._laneToLeft and laneSec._laneToLeft.isForward:
                laneSecsWithAdj.append((laneSec, laneSec._laneToLeft))

# Sample a valid lane configuration
selected_config = Uniform(*laneSecsWithAdj)
egoLaneSec = selected_config[0]
targetLaneSec = selected_config[1]

# Spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Stationary obstacle
obstacle = new Car ahead of egoSpawnPt by globalParameters.OPT_OBSTACLE_DIST,
    with blueprint OBSTACLE_MODEL,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with velocity (0,0,0)

# 2. Lead vehicle
lead_vehicle = new Car ahead of egoSpawnPt by globalParameters.OPT_EGO_FOLLOW_DIST,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior(globalParameters.OPT_LEAD_SPEED, targetLaneSec, obstacle)

# 3. Ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoVehicleBehavior(globalParameters.OPT_EGO_SPEED, obstacle)

# Constraints
require distance to intersection > 50
require (distance from lead_vehicle to obstacle) > (SWERVE_TRIGGER_DIST + 5)

# Safety Termination
terminate when (distance from ego to obstacle < 2 and ego.speed < 0.5)
terminate after 40 seconds