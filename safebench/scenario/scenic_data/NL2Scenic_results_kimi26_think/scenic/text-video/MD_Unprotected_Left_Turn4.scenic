"""Scenario Description:

The ego vehicle executes a left turn at a foggy, urban four-way intersection flanked by tall buildings, viewed from a high-angle perspective that rotates during the sequence. As the ego vehicle moves through the center of the junction, it navigates the turn while two oncoming vehicles from the opposite direction proceed straight through the intersection. Simultaneously, a third vehicle approaches from the left arm and drives straight across the junction, passing perpendicular to the ego vehicle's path. The road is marked with stop lines and crosswalks, and the heavy fog reduces visibility across the multi-lane roads and surrounding cityscape.

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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ONCOMING1_DIST = [15, 20]
ONCOMING2_DIST = [25, 30]
param ONCOMING_SPEED = VerifaiRange(7, 10)

LATERAL_DIST = [15, 20]
param LATERAL_SPEED = VerifaiRange(7, 10)

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

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Straight maneuver from ego's lane used to identify opposite and lateral arms
egoStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

# Two oncoming vehicles from opposite direction proceeding straight
oppositeStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStraightManeuver.reverseManeuvers))
oppositeInitLane = oppositeStraightManeuver.startLane
oppositeTrajectory = [oppositeInitLane, oppositeStraightManeuver.connectingLane, oppositeStraightManeuver.endLane]
oncomingSpawnPt1 = new OrientedPoint in oppositeInitLane.centerline
oncomingSpawnPt2 = new OrientedPoint in oppositeInitLane.centerline

# Third vehicle from left arm driving straight across (perpendicular to ego)
lateralStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStraightManeuver.conflictingManeuvers))
lateralInitLane = lateralStraightManeuver.startLane
lateralTrajectory = [lateralInitLane, lateralStraightManeuver.connectingLane, lateralStraightManeuver.endLane]
lateralSpawnPt = new OrientedPoint in lateralInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

oncoming1 = new Car at oncomingSpawnPt1,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=oppositeTrajectory)

oncoming2 = new Car at oncomingSpawnPt2,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=oppositeTrajectory)

lateralVehicle = new Car at lateralSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LATERAL_SPEED, trajectory=lateralTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ONCOMING1_DIST[0] <= (distance from oncoming1 to intersection) <= ONCOMING1_DIST[1]
require ONCOMING2_DIST[0] <= (distance from oncoming2 to intersection) <= ONCOMING2_DIST[1]
require LATERAL_DIST[0] <= (distance from lateralVehicle to intersection) <= LATERAL_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST