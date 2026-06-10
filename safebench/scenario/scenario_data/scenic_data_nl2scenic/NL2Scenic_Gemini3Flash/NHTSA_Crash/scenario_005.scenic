"""Scenario Description:

Vehicle A (the ego vehicle), in an attempt to turn left at a 4-way intersection, 
cuts the corner too sharply and clips Vehicle B, which is waiting at the 
intersection stop line on the road the ego is turning into.

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

# Using standard car models
EGO_MODEL = 'vehicle.audi.tt'
ADV_MODEL = 'vehicle.bmw.grandtourer'

# Speed and distance constants
param EGO_SPEED = Range(6, 9)
EGO_INIT_DIST = [15, 25]

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitingBehavior():
    """Behavior for Vehicle B to remain stationary at the intersection."""
    while True:
        take SetSpeedAction(0)
        take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Identify a suitable 4-way intersection for the scenario
inter = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# 2. Define Vehicle A (Ego) starting position and trajectory
# Ego starts in an incoming lane and performs a left turn
egoInitLane = Uniform(*inter.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Define Vehicle B (Adversary) position
# Vehicle B is waiting at the intersection entry on the road Ego is turning into.
# We look for incoming lanes to the intersection that belong to the same road as Ego's target lane.
waitingLanes = [l for l in inter.incomingLanes if l.road == egoManeuver.endLane.road]
waitingLane = Uniform(*waitingLanes)

# Position Vehicle B at the stop line (the last point of the incoming lane's centerline)
# This placement ensures the vehicle is at the boundary, making it susceptible to a clipped corner.
advHeading = waitingLane.centerline.headingAt(1.0)
advSpawnPt = waitingLane.centerline.points[-1]

# Define a random spawn point for Ego on its initial lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Vehicle A (Ego)
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

# Spawn Vehicle B (Adversary)
adversary = new Car at advSpawnPt,
    facing advHeading,
    with blueprint ADV_MODEL,
    with behavior WaitingBehavior()

# Ensure Ego starts at the appropriate distance from the intersection to initiate the turn
require EGO_INIT_DIST[0] <= (distance from ego to inter) <= EGO_INIT_DIST[1]

# Terminate scenario once the ego has moved a sufficient distance from its start
terminate when (distance from ego to egoSpawnPt) > 60