"""Scenario Description:

Ego vehicle starts in the left lane of an approach at a 4-way intersection, proceeds straight through the intersection, and executes a lane change into the right lane while traversing the junction. The intersection also features a left-turn path from the left lane and a right-turn path from the right lane.

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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)

TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select the left incoming lane (supports left-turn and straight maneuvers)
egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers) and any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))

# Select the straight maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

# Lane change: from left outgoing lane to right outgoing lane
egoEndLane = egoManeuver.endLane.rightLane
require egoEndLane is not None

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoEndLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST