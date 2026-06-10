"""Scenario Description:
The ego vehicle initiates a driver-requested lane change on a multi-lane road. 
However, it must detect and yield to a high-speed vehicle approaching from the rear 
in the target lane. The ego vehicle delays its maneuver until the overtaking vehicle 
has safely passed and a sufficient gap is established.
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

# Blueprint models
EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.audi.tt'

# Speed constants (m/s)
param EGO_SPEED = Range(7, 9)
param ADV_SPEED = Range(15, 18)

# Distance constants
param INITIAL_ADV_DISTANCE = Range(25, 40)
param SAFETY_GAP_AFTER_PASS = 15
INIT_INTERSECTION_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(targetLaneSec):
    """
    Ego drives, detects a fast vehicle in the target lane,
    waits for it to pass, and then executes a lane change.
    """
    try:
        # Initial driving phase: Ego wants to change lanes but observes the adversary
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until (adversary is behind self)
        
        # Yielding phase: Wait until the high-speed vehicle has passed and reached a safe distance ahead
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) \
            until (adversary is ahead of self) and (distance to adversary > globalParameters.SAFETY_GAP_AFTER_PASS)
        
        # Execution phase: Perform the lane change
        do LaneChangeBehavior(
            laneSectionToSwitchTo=targetLaneSec, 
            target_speed=globalParameters.EGO_SPEED
        )
        
        # Continue in the new lane
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for 10 seconds
        terminate

    interrupt when (distance to intersection) < 10:
        take SetBrakeAction(1.0)

behavior AdversaryBehavior(target_speed):
    """
    Adversary drives at a constant high speed to overtake the ego.
    """
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for road sections that have a valid lane to the left for a lane change
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec.laneToLeft and laneSec.laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

# Select a suitable starting lane section
egoLaneSec = Uniform(*laneSecsWithLeftLane)
targetLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Calculate adversary spawn point behind the ego, projected onto the target lane
advBasePos = egoSpawnPt offset by (0, -globalParameters.INITIAL_ADV_DISTANCE)
advSpawnPos = targetLaneSec.centerline.project(advBasePos)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetLaneSec)

# Spawn the high-speed Adversary in the target lane
adversary = new Car at advSpawnPos,
    facing roadDirection,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure there's enough road ahead for the maneuver
require (distance to intersection) > INIT_INTERSECTION_DIST
# Ensure the adversary starts behind the ego
require adversary is behind ego

# Weather setup
param weather = Uniform('ClearNoon', 'CloudyNoon')

# Safety requirement: no collisions allowed during the simulation
require always (distance from ego to adversary) > 2.0