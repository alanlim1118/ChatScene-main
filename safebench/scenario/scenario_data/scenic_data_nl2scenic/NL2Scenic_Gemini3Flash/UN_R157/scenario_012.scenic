"""Scenario Description:
The ego vehicle follows a lead vehicle in the center of the lane until the lead vehicle 
performs a sudden evasive maneuver to an adjacent lane, revealing a slow-moving target 
motorcycle traveling at a significantly lower speed in the same path. 
The ego vehicle must detect the speed differential and decelerate or brake to avoid 
a collision with the revealed motorcycle.
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
LEAD_MODEL = "vehicle.audi.tt"
TARGET_MODEL = "vehicle.yamaha.yzf"

# Speeds
param EGO_TARGET_SPEED = Range(15, 18)
param LEAD_TARGET_SPEED = globalParameters.EGO_TARGET_SPEED
param MOTORCYCLE_SPEED = Range(3, 5)

# Distances
param EGO_TO_LEAD_DIST = Range(10, 15)
param LEAD_TO_TARGET_DIST = Range(15, 20)
param REVEAL_TRIGGER_DIST = 10  # Lead car changes lane when this close to motorcycle
param BRAKE_THRESHOLD = 15      # Ego starts braking when motorcycle is within this distance

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_speed, reveal_dist, target_lane_section):
    """Follows lane until close to the motorcycle, then performs a sudden lane change."""
    try:
        # Follow lane until the motorcycle is revealed
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to target_motorcycle < reveal_dist)
        
        # Sudden evasive maneuver
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_section, target_speed=target_speed)
        
        # Continue in the new lane
        do FollowLaneBehavior(target_speed=target_speed)
    except:
        # Fallback to standard driving if lane change fails
        do FollowLaneBehavior(target_speed=target_speed)

behavior EgoReactiveBehavior(target_speed, brake_dist):
    """Follows lead vehicle and reacts once the slow motorcycle is revealed."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        # Once the lead vehicle moves and the motorcycle is in the path
        # or if the lead vehicle itself is too close.
        take SetBrakeAction(1.0)
        do ConstantThrottleBehavior(0.0) for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane section that has an adjacent lane for the evasive maneuver
laneSecsWithAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            if laneSec.laneToLeft is not None and laneSec.laneToLeft.isForward:
                laneSecsWithAdjacent.append((laneSec, laneSec.laneToLeft))
            elif laneSec.laneToRight is not None and laneSec.laneToRight.isForward:
                laneSecsWithAdjacent.append((laneSec, laneSec.laneToRight))

if not laneSecsWithAdjacent:
    raise RuntimeError("No suitable multi-lane road found in this map.")

selected_pair = Uniform(*laneSecsWithAdjacent)
egoLaneSec = selected_pair[0]
adjLaneSec = selected_pair[1]

# Spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.EGO_TO_LEAD_DIST
targetSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.LEAD_TO_TARGET_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# The slow-moving motorcycle
target_motorcycle = new Motorcycle at targetSpawnPt,
    with blueprint TARGET_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.MOTORCYCLE_SPEED)

# The lead vehicle that performs the evasive maneuver
lead_car = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior(
        target_speed=globalParameters.LEAD_TARGET_SPEED,
        reveal_dist=globalParameters.REVEAL_TRIGGER_DIST,
        target_lane_section=adjLaneSec
    )

# The ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoReactiveBehavior(
        target_speed=globalParameters.EGO_TARGET_SPEED,
        brake_dist=globalParameters.BRAKE_THRESHOLD
    )

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we are on a straight enough section far from intersections
require distance to intersection > 50
require (distance from target_motorcycle to intersection) > 50

# Terminate when ego has moved significantly or come to a stop after braking
terminate when distance from ego to egoSpawnPt > 150