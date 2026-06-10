"""Scenario Description:

The ego vehicle travels at 100 km/h while a slower-moving target vehicle in the adjacent lane at 60 km/h 
performs a lane change into the ego vehicle's path, forcing the ego vehicle to rapidly bridge 
the speed differential to avoid a rear-end collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05' # Town06 is chosen for its long highway stretches suitable for 100 km/h speeds
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Speed conversions: 100 km/h -> ~27.78 m/s, 60 km/h -> ~16.67 m/s
EGO_SPEED_MS = 27.78 
ADV_SPEED_MS = 16.67

MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.tt'

param TRIGGER_DIST = Range(25, 35) # Distance at which the adversary starts the lane change
param INITIAL_DIST = Range(45, 55) # Initial distance between vehicles
param BRAKE_THRESHOLD = 20         # Distance at which ego starts braking hard
param BRAKE_INTENSITY = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryBehavior(target_lane_section):
    """Adversary drives slower in an adjacent lane and cuts in when ego is close."""
    # Drive at a constant speed in the adjacent lane
    do FollowLaneBehavior(target_speed=ADV_SPEED_MS) until (distance to ego) < globalParameters.TRIGGER_DIST
    
    # Perform lane change into the ego's lane
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_section, target_speed=ADV_SPEED_MS)
    
    # Continue driving at the slower speed to maintain the hazard
    do FollowLaneBehavior(target_speed=ADV_SPEED_MS)

behavior EgoBehavior():
    """Ego drives fast and must brake if the path is blocked."""
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED_MS)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.BRAKE_THRESHOLD):
        # Respond to the cut-in by braking hard
        take SetBrakeAction(BRAKE_INTENSITY)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a lane to their left, allowing a right-to-left or left-to-right cut-in
# We'll set ego on the right and adversary on the left for this reconstruction
validLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Check for a forward lane that has another forward lane to its left
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            validLaneSections.append(laneSec)

# Randomly select a suitable lane section for the scenario
egoLaneSec = Uniform(*validLaneSections)
advLaneSec = egoLaneSec._laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project ego position onto the adversary lane and move forward by INITIAL_DIST
projectedPt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from projectedPt for globalParameters.INITIAL_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(egoLaneSec)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure vehicles are not too close to intersections for highway behavior
require (distance to intersection) > 50
require (distance from adversary to intersection) > 50

# Terminate after a significant distance has been covered or if the scenario is resolved
terminate when (distance to egoSpawnPt) > 300