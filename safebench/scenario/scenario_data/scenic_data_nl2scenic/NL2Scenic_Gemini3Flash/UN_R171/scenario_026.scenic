"""Scenario Description:

A target vehicle (adversary) performs a highly aggressive, high-lateral-acceleration "swerve" 
maneuver to change lanes into the ego vehicle's path. The ego vehicle must respond 
by braking to avoid a collision.

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
ADV_MODEL = "vehicle.audi.tt"  # A smaller, agile car for an aggressive swerve

# Speed and distance parameters
param EGO_SPEED = Range(10, 15)
param ADV_TARGET_SPEED = globalParameters.EGO_SPEED + 5
param SWERVE_TRIGGER_DIST = Range(8, 12)  # Distance at which adversary starts swerving
param BRAKE_THRESHOLD = 10                # Distance at which ego senses the cut-in and brakes
param AGGRESSIVE_SPEED = 20               # High speed during lane change to simulate aggression

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvAggressiveSwerve(target_lane, swerve_speed, normal_speed):
    # Drive in the adjacent lane until the trigger distance is reached
    do FollowLaneBehavior(target_speed=normal_speed) until (distance from self to ego < globalParameters.SWERVE_TRIGGER_DIST)
    
    # Perform the aggressive swerve (lane change)
    # High target speed during the maneuver increases lateral force in the simulation
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=swerve_speed)
    
    # Continue driving after the cut-in
    do FollowLaneBehavior(target_speed=normal_speed)

behavior ReactiveEgoBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, BRAKE_THRESHOLD):
        # Respond to the swerve by braking hard
        take SetBrakeAction(1.0)
        # Hold brake for a moment then terminate for safety
        do ConstantThrottleBehavior(0) for 3 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find lanes that have a lane to their left to allow a right-to-left or left-to-right cut-in
# Here we filter for forward lanes that have a lane to the left
laneSecsWithLeft = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec.laneToLeft and sec.laneToLeft.isForward:
            laneSecsWithLeft.append(sec)

# Select a random valid section
egoLaneSec = Uniform(*laneSecsWithLeft)
# The adversary will start in the lane to the left and swerve into the ego's lane
advLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline
# Adversary spawns ahead of the ego in the adjacent lane
advSpawnPt = new OrientedPoint on advLaneSec.centerline, ahead of egoSpawnPt by Range(10, 15)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego Vehicle ---
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior ReactiveEgoBehavior(globalParameters.EGO_SPEED)

# --- Adversary Vehicle ---
adv = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvAggressiveSwerve(
        egoLaneSec.lane, 
        globalParameters.AGGRESSIVE_SPEED, 
        globalParameters.ADV_TARGET_SPEED
    )

#################################
# CONSTRAINTS                   #
#################################

# Ensure we are on a straight road segment away from intersections for the swerve
require distance to intersection > 50
require (relative heading of adv from ego) < 10 deg

# Termination condition
terminate when (distance from ego to adv) < 5 and ego.speed < 1