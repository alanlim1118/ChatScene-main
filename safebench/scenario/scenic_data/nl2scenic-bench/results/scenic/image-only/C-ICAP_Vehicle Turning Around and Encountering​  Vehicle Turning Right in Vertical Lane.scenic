"""Scenario Description:

This traffic scenario illustrates a four-way intersection on a test road designed with two-way six lanes, comprising three lanes in each direction separated by white dotted lines, while opposing lanes are divided by yellow double solid lines. A blue Vehicle Under Test (VUT) is shown in the leftmost lane of the horizontal road executing a U-turn, indicated by a blue curved path line. A red VUT is positioned in the top vertical lane, facing downward, and is in the process of turning right. The trajectories of the U-turning blue vehicle and the right-turning red vehicle intersect near the top-left corner of the intersection, which is explicitly labeled as the "Estimated collision point." Round traffic lights are situated at the corners of the intersection to regulate traffic flow.

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

BLUE_MODEL = 'vehicle.lincoln.mkz_2017'
RED_MODEL = 'vehicle.tesla.model3'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(5, 8)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
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

# Select a 4-way intersection with sufficient lanes for U-turn and right turn conflict
intersection = Uniform(*filter(lambda i: i.is4Way and len(i.incomingLanes) >= 6, network.intersections))

# Blue VUT: U-turn from leftmost lane of horizontal road
# Find incoming lanes that support U-turn maneuvers
uTurnManeuvers = filter(lambda m: m.type is ManeuverType.U_TURN, intersection.maneuvers)
egoManeuver = Uniform(*uTurnManeuvers)
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Red VUT: Right turn from top vertical lane (conflicting with U-turn)
# Find right turn maneuvers that conflict with the ego's U-turn
conflictingRightTurns = filter(lambda m: 
    m.type is ManeuverType.RIGHT_TURN and m in egoManeuver.conflictingManeuvers, 
    intersection.maneuvers)
advManeuver = Uniform(*conflictingRightTurns)
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue VUT executing U-turn
ego = new Car at egoSpawnPt,
    with blueprint BLUE_MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

# Red VUT executing right turn
adversary = new Car at advSpawnPt,
    with blueprint RED_MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure vehicles start at appropriate distances from intersection
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Terminate after ego has traveled sufficiently past the intersection
terminate when (distance from ego to egoSpawnPt) > TERM_DIST