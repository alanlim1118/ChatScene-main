"""Scenario Description:

Approaching a roundabout, the ego vehicle (green box) is initially positioned in the left lane but intends to execute a right turn, realizing it is in the incorrect lane for this maneuver. Consequently, the ego vehicle performs a lane change into the right lane, directly cutting off an adversary vehicle (yellow box) that is already traveling in that lane. This improper merge forces the ego vehicle into the path of the yellow vehicle as they both near the roundabout entry, creating a conflict where the ego vehicle asserts its position in the right lane to facilitate the turn.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(6, 9)
param LANE_CHANGE_DIST = VerifaiRange(15, 25)
param CUT_OFF_DIST = VerifaiRange(8, 15)
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(leftLane, rightLane, changeDist):
    """Ego starts in left lane, then changes to right lane at specified distance before roundabout."""
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, lane=leftLane) until (distance to roundaboutEntry <= changeDist)
    do LaneChangeBehavior(target_speed=globalParameters.EGO_SPEED, targetLane=rightLane)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, lane=rightLane)

behavior AdversaryBehavior(lane):
    """Adversary follows the right lane toward the roundabout."""
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, lane=lane)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout from the network
roundabout = Uniform(*filter(lambda i: hasattr(i, 'isRoundabout') and i.isRoundabout, network.intersections))

# Get incoming lanes to the roundabout; we need at least two parallel lanes
incomingLanes = list(filter(lambda l: l.successorManeuvers and any(
    m.endLane in roundabout.incomingLanes or m.connectingLane in roundabout.connectingLanes
    for m in l.maneuvers), roundabout.incomingLanes))

# Find a pair of adjacent lanes approaching the roundabout
# leftLane is the outer/left approach lane, rightLane is the inner/right approach lane
lanePair = Uniform(*[
    (l, r) for l in incomingLanes for r in incomingLanes
    if l is not r and l.rightNeighbor is r
])
leftLane = lanePair[0]
rightLane = lanePair[1]

# Define the roundabout entry point as reference for triggering lane change
roundaboutEntry = new OrientedPoint at rightLane.centerline.end

# Ego spawns in the left lane at some distance before the roundabout
egoSpawnPt = new OrientedPoint in leftLane.centerline

# Adversary spawns in the right lane, positioned such that ego will cut it off
# The adversary should be slightly behind or alongside where the ego will merge
advSpawnOffset = Range(-5, 5)  # relative offset along right lane centerline
advSpawnBase = new OrientedPoint in rightLane.centerline
advSpawnPt = new OrientedPoint ahead of advSpawnBase by advSpawnOffset

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 1, 0),  # green box
    with behavior EgoLaneChangeBehavior(leftLane, rightLane, globalParameters.LANE_CHANGE_DIST)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (1, 1, 0),  # yellow box
    with behavior AdversaryBehavior(rightLane)

# Ensure ego starts at a reasonable distance from the roundabout
require 30 <= (distance from ego to roundaboutEntry) <= 60

# Ensure adversary is near the merge zone so the cut-off occurs
require (distance from adversary to roundaboutEntry) <= 40
require (distance from ego to adversary) <= 30

terminate when (distance from ego to egoSpawnPt) > TERM_DIST