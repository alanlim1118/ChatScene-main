"""Scenario Description:

The ego vehicle, represented by a green bounding box, is traveling along a straight multi-lane road section positioned between two four-way intersections. Its forward progress is impeded by a dense cluster of adversary vehicles, indicated by yellow bounding boxes, which are stopped or moving slowly directly ahead and in the adjacent lanes. This accumulation of traffic creates a bottleneck that completely obstructs the ego vehicle's path, effectively forcing it to remain stationary behind the congestion.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
ADV_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.8, 1.0)
param ADV_SPEED = VerifaiRange(0, 2)  # Stopped or very slow

BLOCKING_DIST_AHEAD = Range(15, 25)
BLOCKING_DIST_SIDE = Range(8, 15)
SAFETY_DIST = 12
CRASH_DIST = 4
TERM_TIME = 30  # Terminate after enough time to observe stationary behavior

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior SlowOrStoppedBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find road sections between two 4-way intersections
fourWayIntersections = list(filter(lambda i: i.is4Way, network.intersections))

# Collect lane sections that are on straight roads between intersections
candidateLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and not sec.isInIntersection:
            # Check that this section is between intersections by verifying
            # it has both a predecessor and successor leading toward intersections
            candidateLaneSections.append(sec)

egoLaneSec = Uniform(*candidateLaneSections)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversary directly ahead in same lane
advAheadOffset = BLOCKING_DIST_AHEAD
advAheadPt = new OrientedPoint following roadDirection from egoSpawnPt for advAheadOffset

# Adversary in left adjacent lane (if exists)
hasLeft = egoLaneSec._laneToLeft is not None and egoLaneSec._laneToLeft.isForward
leftLaneSec = egoLaneSec._laneToLeft if hasLeft else None
advLeftPt = new OrientedPoint in leftLaneSec.centerline,
    with visibleDistance 50
require (distance from advLeftPt to egoSpawnPt) <= BLOCKING_DIST_SIDE + 20 if hasLeft else True

# Adversary in right adjacent lane (if exists)
hasRight = egoLaneSec._laneToRight is not None and egoLaneSec._laneToRight.isForward
rightLaneSec = egoLaneSec._laneToRight if hasRight else None
advRightPt = new OrientedPoint in rightLaneSec.centerline,
    with visibleDistance 50
require (distance from advRightPt to egoSpawnPt) <= BLOCKING_DIST_SIDE + 20 if hasRight else True

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 1, 0),  # Green bounding box
    with regionContainedIn egoLaneSec,
    with behavior EgoBehavior()

# Adversary directly ahead
advAhead = new Car at advAheadPt,
    with blueprint ADV_MODEL,
    with color (1, 1, 0),  # Yellow bounding box
    with regionContainedIn egoLaneSec,
    with behavior SlowOrStoppedBehavior()

# Adversary in left lane
if hasLeft:
    advLeft = new Car at advLeftPt,
        with blueprint ADV_MODEL,
        with color (1, 1, 0),
        with regionContainedIn leftLaneSec,
        with behavior SlowOrStoppedBehavior()

# Adversary in right lane
if hasRight:
    advRight = new Car at advRightPt,
        with blueprint ADV_MODEL,
        with color (1, 1, 0),
        with regionContainedIn rightLaneSec,
        with behavior SlowOrStoppedBehavior()

# Ensure ego is between intersections (not too close to either)
require distance from ego to intersection >= 30

# Ensure adversaries are positioned ahead and beside ego
require (distance from advAhead to ego) <= BLOCKING_DIST_AHEAD[1]
require (distance from advAhead to ego) >= BLOCKING_DIST_AHEAD[0]

terminate after TERM_TIME seconds