"""Scenario Description:

The ego vehicle travels in the left lane of an urban street under clear weather conditions, approaching a four-way intersection situated beneath a large concrete overpass structure. Intending to make a right turn, the ego vehicle initiates a lane change toward the right lane. However, the target right lane is occupied by a line of three adversary vehicles—specifically a red car followed by two grey vehicles further ahead—traveling straight through the intersection. Consequently, the ego vehicle is forced to manage its spacing and yield to the oncoming traffic in the right lane before it can complete the lane change and execute the turn.

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

EGO_INIT_DIST = [25, 35]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

# Adversary distances from intersection (red closest to ego, grey vehicles further ahead)
RED_DIST = [18, 23]
GREY1_DIST = [12, 17]
GREY2_DIST = [6, 11]

param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 70

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

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Identify right lane via a right-turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN,
    [m for lane in intersection.incomingLanes for m in lane.maneuvers]))
rightLane = egoManeuver.startLane

# Get adjacent left lane from the same lane group
laneGroup = rightLane.laneGroup
laneIdx = laneGroup.lanes.index(rightLane)
require laneIdx > 0
leftLane = laneGroup.lanes[laneIdx - 1]

# Ego: starts in left lane, follows right-turn trajectory (forces lane change)
egoTrajectory = [rightLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in leftLane.centerline

# Adversaries: in right lane, traveling straight
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, rightLane.maneuvers))
advTrajectory = [rightLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points for the three adversaries in the right lane
advSpawnPt1 = new OrientedPoint in rightLane.centerline  # red car
advSpawnPt2 = new OrientedPoint in rightLane.centerline  # grey vehicle 1
advSpawnPt3 = new OrientedPoint in rightLane.centerline  # grey vehicle 2

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at advSpawnPt1,
    with blueprint MODEL,
    with color [0.8, 0.1, 0.1],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary2 = new Car at advSpawnPt2,
    with blueprint MODEL,
    with color [0.5, 0.5, 0.5],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary3 = new Car at advSpawnPt3,
    with blueprint MODEL,
    with color [0.5, 0.5, 0.5],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Distance constraints to order vehicles before the intersection
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require RED_DIST[0] <= (distance from adversary1 to intersection) <= RED_DIST[1]
require GREY1_DIST[0] <= (distance from adversary2 to intersection) <= GREY1_DIST[1]
require GREY2_DIST[0] <= (distance from adversary3 to intersection) <= GREY2_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST