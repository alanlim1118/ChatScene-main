"""Scenario Description:

The ego vehicle is driving in a lane that is about to be reduced. 
A signboard (TrafficWarning) is positioned in the center of the lane to notify drivers of the lane reduction.
The ego vehicle detects the signboard and performs a lane change to the adjacent left lane to avoid the obstruction.

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
param EGO_SPEED = Range(10, 15)
param SIGN_DISTANCE = Range(50, 70)          # Distance from ego spawn to the signboard
param THRESHOLD_DISTANCE = Range(25, 35)    # Distance at which ego decides to change lanes

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed, target_lane, threshold):
    """Ego drives forward and changes lanes when a signboard is detected."""
    try:
        # Drive until the signboard is within the threshold distance
        do FollowLaneBehavior(target_speed=target_speed) until withinDistanceToAnyObjs(self, threshold)
        
        # Perform lane change to the specified adjacent lane
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=target_speed)
        
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, 5):
        # Basic safety interrupt
        take SetBrakeAction(1.0)
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that have an adjacent lane to the left (to simulate moving out of a closing right lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec.laneToLeft is not None and laneSec.laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

# Select a starting lane section and its left neighbor
egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Place the signboard ahead of the ego in the center of the same lane
signPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.SIGN_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Signboard (TrafficWarning) in the center of the lane
signboard = new TrafficWarning at signPt,
    with heading signPt.heading

# Spawn the Ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED, 
        adjLaneSec, 
        globalParameters.THRESHOLD_DISTANCE
    )

# Ensure the scenario starts on a straight road segment away from immediate intersections
require distance to intersection >= 20
require 40 <= (distance from ego to signboard) <= 80

# Terminate the simulation once the ego has safely passed the signboard in the other lane
terminate when (distance from ego to signboard < 10) and (ego.laneSection == adjLaneSec)