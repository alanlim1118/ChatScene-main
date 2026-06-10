"""Scenario Description:

Vehicle A (Ego), in an attempt to turn left at a 4-way intersection, cuts the corner too sharply 
and clips Vehicle B (Adversary) waiting at the intersection. 
Vehicle A begins the turn too early and misjudges the distance between the cars.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = Range(6, 8)
param ADV_SPEED = 0

# Distance constants
EGO_START_DIST = Range(20, 25)
TURN_THRESHOLD = 8  # Distance to intersection when Ego starts turning early

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitingBehavior():
    """Behavior for Vehicle B, waiting at the intersection stop line."""
    take SetBrakeAction(1.0)
    while True:
        take SetSpeedAction(0)

behavior SharpTurnBehavior(trajectory):
    """Behavior for Vehicle A, cutting the corner early."""
    # Drive towards the intersection
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until (distance to intersection) < TURN_THRESHOLD
    
    # "Cut the corner" by starting the trajectory/turning behavior early 
    # and shifting the path slightly to the inside of the turn.
    self.roadDeviation = -1.2  # Shift to the left (inside of a left turn)
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, 0.1):
        # Stop upon clipping Vehicle B
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Identify a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# 2. Define Vehicle A's (Ego) path: A left turn
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Define Vehicle B's (Adversary) position
# Vehicle B should be waiting at the stop line of the road Ego is turning into.
# This corresponds to an incoming lane from the road that Ego's maneuver ends on.
targetRoad = egoManeuver.endLane.road
advLanes = filter(lambda l: l in intersection.incomingLanes and l.road == targetRoad, network.lanes)
advLane = Uniform(*advLanes)

# Spawn point for B is at the very end of its lane (at the stop line)
advSpawnPt = advLane.centerline.points[-1]

# Spawn point for Ego is back from the intersection
egoSpawnPt = new OrientedPoint on egoInitLane.centerline,
    beyond intersection by -globalParameters.EGO_START_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Vehicle B (Adversary) - the one waiting
adversary = new Car at advSpawnPt,
    facing advLane.orientation,
    with blueprint MODEL,
    with behavior WaitingBehavior()

# Spawn Vehicle A (Ego) - the one turning sharply
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior SharpTurnBehavior(egoTrajectory)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure Ego is actually approaching the intersection
require (distance to intersection) <= 30

# Terminate scenario if they move far past the intersection or after collision
terminate when (distance to egoSpawnPt) > 60