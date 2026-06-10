"""Scenario Description:
The ego vehicle follows a lead M1 category vehicle at a specific distance. 
The lead vehicle suddenly performs a 3.5-meter lane change (shifting to an adjacent lane) 
at a Time-to-Collision (TTC) of less than 3 seconds relative to a stationary target 
vehicle of a different category (Truck) directly in the ego vehicle's path. 
This reveals the obstacle, requiring the ego vehicle to respond while the lead vehicle 
maintains its trajectory.
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

EGO_MODEL = "vehicle.tesla.model3"
LEAD_MODEL = "vehicle.audi.tt"
OBSTACLE_MODEL = "vehicle.carlamotors.carlacola"

param EGO_SPEED = Range(15, 18)
param LEAD_SPEED = globalParameters.EGO_SPEED
param INITIAL_GAP = Range(15, 20)
param OBSTACLE_DIST = Range(60, 80)
param TTC_THRESHOLD = 3.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(target_lane, stationary_obj):
    # Drive ahead of ego
    try:
        do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)
    interrupt when (distance from ego to stationary_obj) / ego.speed < globalParameters.TTC_THRESHOLD:
        # Perform sudden lane change to reveal the obstacle
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=globalParameters.LEAD_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior EgoBehavior():
    # Ego follows lane and must avoid collision once obstacle is revealed
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, 20):
        # Emergency braking once the truck is revealed in its lane
        take SetBrakeAction(1.0)

behavior StationaryBehavior():
    while True:
        take SetSpeedAction(0)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane that has an adjacent lane for the lead vehicle to escape into
laneSecsWithAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            # Check for a lane to the left or right
            if laneSec._laneToLeft and laneSec._laneToLeft.isForward:
                laneSecsWithAdjacent.append((laneSec, laneSec._laneToLeft))
            elif laneSec._laneToRight and laneSec._laneToRight.isForward:
                laneSecsWithAdjacent.append((laneSec, laneSec._laneToRight))

if not laneSecsWithAdjacent:
    raise RuntimeError("No suitable lanes found with adjacent lanes for the scenario.")

selectedLanes = Uniform(*laneSecsWithAdjacent)
egoLaneSec = selectedLanes[0]
escapeLaneSec = selectedLanes[1]

# Define Spawn Points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.INITIAL_GAP
obstacleSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OBSTACLE_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# The stationary obstacle (Truck - different category)
obstacle = new Truck at obstacleSpawnPt,
    with blueprint OBSTACLE_MODEL,
    with behavior StationaryBehavior()

# The ego vehicle (M1 category)
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# The lead vehicle (M1 category)
lead_veh = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior(escapeLaneSec, obstacle)

# Constraints
require (distance to intersection) > 50
require (distance from obstacle to intersection) > 20

# Termination
terminate when ego.speed < 0.1 and distance to obstacle < 15
terminate when (distance from ego to egoSpawnPt) > globalParameters.OBSTACLE_DIST + 20