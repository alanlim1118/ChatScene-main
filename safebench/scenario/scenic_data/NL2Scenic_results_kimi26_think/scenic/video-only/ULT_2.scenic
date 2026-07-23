"""Scenario Description:

In a dense urban canyon formed by tall skyscrapers, the ego vehicle travels along a city street approaching a signalized four-way intersection under clear daylight conditions. The ego vehicle follows a red lead vehicle that is executing a left turn through the junction. As the ego vehicle enters the intersection to complete the same left turn, it must navigate around adversary vehicles that are simultaneously entering from the opposing lane and the right-hand cross-street to proceed straight. The scenario requires the ego vehicle to maintain a safe following distance behind the red car while carefully monitoring the conflicting straight-moving traffic to ensure a safe passage through the intersection.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [25, 35]
LEAD_INIT_DIST = [10, 20]
param EGO_SPEED = VerifaiRange(5, 8)
param LEAD_SPEED = VerifaiRange(4, 7)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_OPP_INIT_DIST = [15, 25]
ADV_RIGHT_INIT_DIST = [15, 25]
param ADV_OPP_SPEED = VerifaiRange(7, 10)
param ADV_RIGHT_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 5
TERM_DIST = 80

MIN_FOLLOW_DIST = 5
MAX_FOLLOW_DIST = 15

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

# Ego and lead both execute the same left-turn maneuver
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

leadInitLane = egoInitLane
leadManeuver = egoManeuver
leadTrajectory = egoTrajectory

# Two conflicting straight maneuvers: opposing lane and right-hand cross-street
conflictingStraights = list(filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advOppManeuver = Uniform(*conflictingStraights)
advRightManeuver = Uniform(*filter(lambda m: m != advOppManeuver, conflictingStraights))

advOppInitLane = advOppManeuver.startLane
advOppTrajectory = [advOppInitLane, advOppManeuver.connectingLane, advOppManeuver.endLane]

advRightInitLane = advRightManeuver.startLane
advRightTrajectory = [advRightInitLane, advRightManeuver.connectingLane, advRightManeuver.endLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

leadSpawnPt = new OrientedPoint in leadInitLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advOppSpawnPt = new OrientedPoint in advOppInitLane.centerline
advRightSpawnPt = new OrientedPoint in advRightInitLane.centerline

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with color [255, 0, 0],
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=leadTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary_opposing = new Car at advOppSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_OPP_SPEED, trajectory=advOppTrajectory)

adversary_right = new Car at advRightSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_RIGHT_SPEED, trajectory=advRightTrajectory)

#################################
# REQUIREMENTS                  #
#################################

# Ensure lead is ahead of ego at a safe following distance
require LEAD_INIT_DIST[0] <= (distance from lead to intersection) <= LEAD_INIT_DIST[1]
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require (distance from lead to intersection) < (distance to intersection)
require ((distance to intersection) - (distance from lead to intersection)) >= MIN_FOLLOW_DIST
require ((distance to intersection) - (distance from lead to intersection)) <= MAX_FOLLOW_DIST

# Adversary distance requirements
require ADV_OPP_INIT_DIST[0] <= (distance from adversary_opposing to intersection) <= ADV_OPP_INIT_DIST[1]
require ADV_RIGHT_INIT_DIST[0] <= (distance from adversary_right to intersection) <= ADV_RIGHT_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST