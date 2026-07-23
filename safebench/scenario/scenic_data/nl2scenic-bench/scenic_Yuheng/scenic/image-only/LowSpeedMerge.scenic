"""Scenario Description:

An autonomous driving system (ADS) equipped vehicle, depicted in green, travels in the left lane of a straight, multi-lane urban road and initiates a lane change maneuver to the right lane to prepare for a necessary turn. The target lane is occupied by two other vehicles, one positioned ahead and another behind, creating a specific gap between them. A vertical double-headed arrow labeled "Desired Merge Location" highlights this gap as the intended destination for the merging vehicle. The diagram illustrates a test scenario where the ADS vehicle must safely navigate into the space between the leading and trailing vehicles in the adjacent lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(6, 9)
param OPT_LEAD_VEH_SPEED = globalParameters.OPT_EGO_SPEED + Range(0.5, 2.0)   # Leading vehicle in target lane slightly faster or matching
param OPT_TRAIL_VEH_SPEED = globalParameters.OPT_EGO_SPEED - Range(0.5, 2.0)  # Trailing vehicle in target lane slightly slower
param OPT_GAP_CENTER_DIST = Range(30, 50)       # Distance ahead of ego spawn where the gap center is located
param OPT_GAP_SIZE = Range(25, 40)              # Total gap size between lead and trail vehicles
param OPT_LANE_CHANGE_TRIGGER_DIST = Range(15, 25)  # Distance to gap center before initiating lane change
param OPT_SAFETY_BRAKE_DIST = Range(4, 7)       # Minimum safe distance to brake during merge

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    # Follow current lane until close enough to the desired merge location
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (
        distance from self to GapCenterPoint < globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST
    )
    # Initiate lane change to the right lane
    try:
        do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_SAFETY_BRAKE_DIST):
        take SetBrakeAction(1)
    # Continue in the right lane after merge
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior LeadVehBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_VEH_SPEED)

behavior TrailVehBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRAIL_VEH_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid right neighbor (since ego starts in left lane and merges right)
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point in the left lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Define the desired merge location (gap center) projected onto the right lane
rightLaneRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
GapCenterPoint = new OrientedPoint following roadDirection from rightLaneRefPt for globalParameters.OPT_GAP_CENTER_DIST

# Position lead and trail vehicles relative to the gap center
halfGap = globalParameters.OPT_GAP_SIZE / 2.0
LeadVehSpawnPt = new OrientedPoint following roadDirection from GapCenterPoint for halfGap
TrailVehSpawnPt = new OrientedPoint following roadDirection from GapCenterPoint for -halfGap

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in the left lane (green ADS vehicle)
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 255, 0),
    with behavior EgoBehavior()

# Leading vehicle in the right lane (ahead of the gap)
LeadVehicle = new Car at LeadVehSpawnPt,
    with heading LeadVehSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior LeadVehBehavior()

# Trailing vehicle in the right lane (behind the gap)
TrailVehicle = new Car at TrailVehSpawnPt,
    with heading TrailVehSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior TrailVehBehavior()

# Ensure sufficient road length and no intersections nearby
require distance to intersection >= 100

# Terminate after ego has completed the merge and traveled some distance
terminate when (distance from ego to egoSpawnPt > 120)