"""Scenario Description:

The ego vehicle attempts to initiate a lane change while a parallel passenger car maintains a steady position 
directly in the "blind spot" or adjacent zone, requiring the ego vehicle to maintain its lane and wait 
for the adjacent vehicle to either accelerate or fall back before executing the lateral shift.

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
ADV_MODEL = "vehicle.audi.a2"

param OPT_SPEED = Range(7, 10)  # Shared target speed to keep vehicles parallel
param OPT_BLIND_SPOT_DISTANCE = Range(-2, 2) # Longitudinal offset to place ADV in ego's blind spot
param OPT_LANE_CHANGE_TRIGGER_TIME = Range(3, 5) # Seconds before ego tries to change lanes
param OPT_SAFE_DISTANCE = 15 # Minimum distance required in target lane to safely change

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior(target_speed):
    # The adversary simply drives straight, maintaining the blocking position
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoBehavior(target_speed, target_lane_sec):
    # Drive for a few seconds before attempting the maneuver
    do FollowLaneBehavior(target_speed=target_speed) for globalParameters.OPT_LANE_CHANGE_TRIGGER_TIME seconds
    
    # Attempt lane change, but abort/interrupt if the adversary is blocking the path
    try:
        # Checking if the target lane is clear
        if withinDistanceToAnyCars(self, globalParameters.OPT_SAFE_DISTANCE):
            # If a car is detected in the adjacent zone, "wait" by continuing in current lane
            do FollowLaneBehavior(target_speed=target_speed)
        else:
            do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed)
    
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_SAFE_DISTANCE) and (AdvAgent.laneSection == target_lane_sec):
        # Specific logic to handle the "wait" requirement: maintain lane while blocked
        take SetBrakeAction(0.1) # Slight adjustment to maintain safety
        do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that have a lane to their left (both moving in the same direction)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

# Selection of a random suitable lane section
egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

# Define starting positions
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project a point into the adjacent lane for the adversary, incorporating a longitudinal offset
# to simulate the "blind spot" (parallel or slightly behind/ahead)
advBasePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advBasePt for globalParameters.OPT_BLIND_SPOT_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# The adversarial vehicle maintains a steady position in the adjacent lane
AdvAgent = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with heading egoSpawnPt.heading,
    with behavior AdvBehavior(globalParameters.OPT_SPEED)

# The ego vehicle tries to merge but is blocked
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_SPEED, adjLaneSec)

#################################
# REQUIREMENTS                  #
#################################

# Ensure there is enough road ahead for the maneuver to play out
require distance to intersection > 80
require (distance from ego to AdvAgent) < 10

# Terminate after a reasonable simulation time
terminate when simulation().currentTime > 15