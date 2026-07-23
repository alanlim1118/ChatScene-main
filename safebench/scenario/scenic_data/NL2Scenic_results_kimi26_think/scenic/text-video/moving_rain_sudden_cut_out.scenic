"""Scenario Description:

The ego vehicle travels forward on a wet, divided highway during rainy conditions with active windshield wipers clearing reduced visibility. A dark-colored SUV drives directly ahead in the same lane before abruptly illuminating its brake lights and decelerating sharply. In response to a stationary obstacle further ahead, the lead vehicle suddenly swerves into the right lane to avoid a collision. This unexpected maneuver forces the ego vehicle into an emergency braking and evasive swerving sequence, ultimately causing the ego vehicle to veer right and collide with the lead vehicle as it attempts to navigate the hazardous, low-traction environment.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_MODEL = "vehicle.audi.etron"  # SUV

param OPT_EGO_SPEED = Range(12, 16)
param OPT_LEAD_SPEED = Range(12, 16)
param OPT_LEAD_DIST = Range(20, 25)          # Distance from ego to lead vehicle
param OPT_OBSTACLE_DIST = Range(50, 70)      # Distance from lead vehicle to stationary obstacle
param OPT_BRAKE_DIST = Range(20, 30)         # Distance at which lead reacts to obstacle
param OPT_EGO_REACT_DIST = Range(12, 18)     # Distance at which ego reacts to lead vehicle

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior LeadBehavior(lead_speed, obstacle, right_lane):
    do FollowLaneBehavior(target_speed=lead_speed) until (distance from self to obstacle < globalParameters.OPT_BRAKE_DIST)
    take SetBrakeAction(1.0)  # Abruptly illuminate brake lights and decelerate
    do LaneChangeBehavior(laneSectionToSwitch=right_lane, target_speed=lead_speed * 0.5)
    do FollowLaneBehavior(target_speed=lead_speed)

behavior EgoBehavior(ego_speed, lead_vehicle, right_lane):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to lead_vehicle < globalParameters.OPT_EGO_REACT_DIST):
        take SetBrakeAction(1.0)  # Emergency braking
        do LaneChangeBehavior(laneSectionToSwitch=right_lane, target_speed=ego_speed * 0.5)  # Evasive swerve right
        do FollowLaneBehavior(target_speed=ego_speed)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
rightLaneSec = egoLaneSec._laneToRight

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST
obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_OBSTACLE_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Stationary obstacle further ahead in the same lane ---
StationaryObstacle = new Car at obstacleSpawnPt,
    with regionContainedIn egoLaneSec,
    with behavior WaitBehavior()

# --- Lead vehicle (dark-colored SUV) directly ahead in the same lane ---
LeadAgent = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint LEAD_MODEL,
    with color (0.1, 0.1, 0.1),  # Dark colored
    with behavior LeadBehavior(
        globalParameters.OPT_LEAD_SPEED,
        StationaryObstacle,
        rightLaneSec
    )

# --- Ego vehicle ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        LeadAgent,
        rightLaneSec
    )

require distance to intersection >= 100  # Ensure the scenario plays out on the highway away from intersections
terminate when ego.speed < 1 and (distance to LeadAgent) < 10