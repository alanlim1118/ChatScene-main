"""Scenario Description:

The blue ego vehicle is traveling in the right lane and executing a lane change maneuver to the left lane. Ahead of the ego vehicle in the right lane, a leading pink car is simultaneously merging into the same left target lane. At the same time, a trailing pink car is traveling in the left target lane, approaching from behind the ego vehicle's current position, requiring the ego vehicle to coordinate its merge safely between the leading merging vehicle and the trailing vehicle in the target lane.

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
PINK_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(6, 9)
param OPT_LEADING_SPEED = Range(6, 9)
param OPT_TRAILING_SPEED = Range(8, 12)

param OPT_LEADING_DIST_AHEAD = Range(25, 40)       # Leading car distance ahead of ego in right lane
param OPT_TRAILING_DIST_BEHIND = Range(20, 35)     # Trailing car distance behind ego in left lane
param OPT_EGO_LANE_CHANGE_DIST = Range(15, 25)     # Distance at which ego initiates lane change
param OPT_LEADING_MERGE_DIST = Range(18, 28)       # Distance at which leading car initiates merge
param OPT_SAFE_DISTANCE = Range(8, 12)             # Minimum safe distance for braking
param OPT_GEO_X_OFFSET = Range(-3.5, -3.0)         # Lateral offset to left lane centerline

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to LeadingCar < globalParameters.OPT_EGO_LANE_CHANGE_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to LeadingCar < globalParameters.OPT_SAFE_DISTANCE or distance from self to TrailingCar < globalParameters.OPT_SAFE_DISTANCE):
        take SetBrakeAction(1)

behavior LeadingCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED) until (distance from self to ego < globalParameters.OPT_LEADING_MERGE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=leadingLaneSec._laneToLeft, is_oppositeTraffic=False, target_speed=globalParameters.OPT_LEADING_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

behavior TrailingCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRAILING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a valid left lane for merging
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward
        ):
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point in right lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Leading car spawn point ahead of ego in the same right lane
leadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST_AHEAD
leadingLaneSec = network.laneSectionAt(leadingSpawnPt)

# Trailing car spawn point behind ego in the left target lane
trailingOffset = globalParameters.OPT_GEO_X_OFFSET @ (-globalParameters.OPT_TRAILING_DIST_BEHIND)
trailingSpawnPt = new OrientedPoint at egoSpawnPt offset along egoSpawnPt.heading by trailingOffset,
    with heading egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior()

LeadingCar = new Car at leadingSpawnPt,
    with heading leadingSpawnPt.heading,
    with regionContainedIn None,
    with blueprint PINK_MODEL,
    with color (1, 0.4, 0.7),
    with behavior LeadingCarBehavior()

TrailingCar = new Car at trailingSpawnPt,
    with heading trailingSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint PINK_MODEL,
    with color (1, 0.4, 0.7),
    with behavior TrailingCarBehavior()

require distance to intersection >= 100
require leadingLaneSec is egoLaneSec  # Ensure leading car is in same lane section as ego
terminate when (distance from ego to egoSpawnPt > 120)