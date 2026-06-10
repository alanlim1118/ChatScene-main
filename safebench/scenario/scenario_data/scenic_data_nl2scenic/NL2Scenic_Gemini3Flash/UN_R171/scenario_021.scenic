"""Scenario Description:
The lead vehicle executes a highly aggressive, high-lateral-acceleration lane change to simulate 
a panic swerve, testing whether the ego vehicle's sensors can maintain a stable track on the 
revealed stationary object despite the rapid visual occlusion change.
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
LEAD_MODEL = "vehicle.audi.etron"
OBSTACLE_MODEL = "vehicle.volkswagen.t2" # A stalled van

param OPT_SPEED = Range(12, 15)
param OPT_EGO_FOLLOW_DIST = Range(15, 20)
param OPT_LEAD_TO_OBSTACLE_DIST = Range(25, 30)
param OPT_SWERVE_THRESHOLD = Range(12, 15) # Distance at which lead vehicle initiates swerve

#################################
# AGENT BEHAVIORS               #
#################################

behavior PanicSwerveBehavior(target_lane_sec, swerve_dist, target_speed):
    """
    Lead vehicle drives normally until it is close to the obstacle,
    then performs an aggressive lane change to simulate a swerve.
    """
    # Follow lane until the obstacle is close
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to obstacle < swerve_dist)
    
    # Execute the 'panic' swerve
    # Increasing target_speed slightly during lane change to simulate aggressive maneuver
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed * 1.2)
    
    # Continue in the new lane
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoFollowingBehavior(target_speed, lead_obj):
    """
    Ego follows the lane, attempting to maintain speed but reacting to the environment.
    """
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, 10):
        # Emergency braking if the revealed obstacle is too close
        take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lane sections that have a lane to the left to swerve into
laneSecsWithLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeft.append(laneSec)

assert len(laneSecsWithLeft) > 0, "No suitable multi-lane sections found in Town05"

# Select a starting lane section
egoLaneSec = Uniform(*laneSecsWithLeft)
targetLaneSec = egoLaneSec._laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. The revealed stationary object
obstacle = new NPCCar following roadDirection from egoSpawnPt for (globalParameters.OPT_EGO_FOLLOW_DIST + globalParameters.OPT_LEAD_TO_OBSTACLE_DIST),
    with blueprint OBSTACLE_MODEL,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec

# 2. The Lead Vehicle (the swerving actor)
leadVehicle = new Car following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_FOLLOW_DIST,
    with blueprint LEAD_MODEL,
    with behavior PanicSwerveBehavior(
        targetLaneSec, 
        globalParameters.OPT_SWERVE_THRESHOLD, 
        globalParameters.OPT_SPEED
    )

# 3. The Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoFollowingBehavior(globalParameters.OPT_SPEED, leadVehicle)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we are on a reasonably long road stretch
require (distance from ego to intersection) > 50
require (distance from obstacle to intersection) > 20

# Terminate when ego has passed the obstacle or stopped
terminate when (distance from ego to obstacle) < 2 and ego.speed < 1
terminate when (relative heading of obstacle.position from ego) > 90 deg