"""Scenario Description:

From a top-down perspective, a multi-lane road runs vertically between a set of railway tracks on the left and a grassy area with trees on the right, leading towards a T-intersection at the top. A blue vehicle travels straight in the left lane, while a pink ego vehicle in the right lane initiates a diagonal lane change to the left. The pink vehicle's trajectory cuts across the lane divider directly in front of the blue car, effectively merging into the left lane to continue straight through the upcoming junction, thereby cutting off the path of the blue vehicle traveling in that lane.

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

MODEL = 'vehicle.lincoln.mkz_2017'

# Ego (pink) is ahead in the right lane, closer to the intersection
EGO_INIT_DIST = [8, 12]
param EGO_SPEED = VerifaiRange(7, 10)

# Adversary (blue) is behind in the left lane
ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-4-way intersection to approximate a T-intersection
intersection = Uniform(*filter(lambda i: not i.is4Way, network.intersections))

# Select a straight maneuver through the intersection
maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))

# Left lane for the straight-through path
leftLane = maneuver.startLane

# Adjacent lane to the right (same road, next lane over)
rightLane = leftLane.rightLane
require rightLane is not None

# Shared straight-through trajectory
trajectory = [leftLane, maneuver.connectingLane, maneuver.endLane]

# Spawn points
egoSpawnPt = new OrientedPoint in rightLane.centerline
advSpawnPt = new OrientedPoint in leftLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Pink ego vehicle: starts in the right lane and follows the left-lane trajectory,
# causing a diagonal cut across the lane divider in front of the blue car.
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color [1.0, 0.4, 0.7],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

# Blue adversary vehicle: travels straight in the left lane.
adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color [0.0, 0.0, 1.0],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

# Ensure ego is ahead (closer to intersection) and adversary is behind
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require (distance to intersection) < (distance from adversary to intersection)

terminate when (distance to egoSpawnPt) > TERM_DIST