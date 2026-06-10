"""Scenario Description:

The ego vehicle is cruising at a constant speed in its lane. A large heavy-duty truck 
in the adjacent lane (to the left) suddenly executes a lane change into the ego vehicle's 
lane, forcing the ego vehicle to react.

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
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param EGO_SPEED = Range(12, 15)
param TRUCK_SPEED = Range(9, 11)
param CUT_IN_SPEED = 12

# The distance ahead the truck starts relative to the ego
param INITIAL_GAP = Range(15, 25)

# The distance threshold that triggers the truck's lane change
param TRIGGER_DISTANCE = Range(15, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior TruckBehavior(target_lane_sec):
    # Drive in the adjacent lane until the ego vehicle is close enough
    try:
        do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to ego < globalParameters.TRIGGER_DISTANCE)
    
        # Execute the sudden lane change
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=globalParameters.CUT_IN_SPEED)
        
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)
    
    interrupt when withinDistanceToAnyCars(self, 5):
        # Basic safety if something goes wrong during the transition
        take SetBrakeAction(1.0)

behavior EgoBehavior():
    # Ego cruises at a constant speed, but avoids collision if the truck cuts in
    do DriveAvoidingCollisions(target_speed=globalParameters.EGO_SPEED, avoidance_threshold=8)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lane sections that have a valid adjacent lane to the right 
# (Truck will be in the left lane, Ego in the right lane)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

assert len(laneSecsWithRightLane) > 0, "No suitable multi-lane road found in this map."

# Select a random valid section
truckLaneSec = Uniform(*laneSecsWithRightLane)
egoLaneSec = truckLaneSec._laneToRight

# Define spawn points
# Ego starts at a point, Truck starts ahead in the adjacent lane
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline
truckSpawnPt = new OrientedPoint following roadDirection from (truckLaneSec.centerline.project(egoSpawnPt.position)) for globalParameters.INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

truck = new Truck at truckSpawnPt,
    with blueprint TRUCK_MODEL,
    with behavior TruckBehavior(egoLaneSec)

# Ensure the scenario happens on a straight road segment away from intersections
require (distance to intersection) > 40
require (distance from truck to intersection) > 40

# Terminate the scenario after some movement to ensure the interaction is captured
terminate when (distance from ego to egoSpawnPt) > 150