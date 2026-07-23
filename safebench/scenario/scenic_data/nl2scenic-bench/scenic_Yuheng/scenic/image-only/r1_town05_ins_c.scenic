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

EGO_INIT_DIST = [30, 40]
param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [35, 45]
param ADV_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a T-intersection where we can set up the scenario
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego starts in the right incoming lane and performs a lane change to the left lane, then goes straight
egoInitLane = Uniform(*filter(lambda l: l is not l.leftLane and l.leftLane is not None, intersection.incomingLanes))
egoLaneChangeManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LANE_CHANGE_LEFT, egoInitLane.maneuvers))
egoTargetLane = egoLaneChangeManeuver.endLane
egoStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoTargetLane.maneuvers))
egoTrajectory = [egoInitLane, egoLaneChangeManeuver.connectingLane, egoTargetLane, egoStraightManeuver.connectingLane, egoStraightManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary (blue car) starts in the left lane (which is the target lane of ego's lane change) and goes straight
advInitLane = egoTargetLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Pink ego vehicle in the right lane performing lane change to cut off blue car
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (1.0, 0.4, 0.7),
    with behavior EgoBehavior(egoTrajectory)

# Blue adversary vehicle traveling straight in the left lane
adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (0.2, 0.3, 0.9),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure proper initial distances from intersection
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure adversary is ahead of or alongside ego so the cut-off occurs in front of the blue car
require (distance from ego to adversary) >= 0

terminate when (distance to egoSpawnPt) > TERM_DIST