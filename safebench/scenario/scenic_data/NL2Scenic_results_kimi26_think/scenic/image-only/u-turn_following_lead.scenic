"""Scenario Description:

Two vehicles approach a 4-way intersection on the same road. A pink vehicle in the left lane performs a U-turn maneuver angled into the intersection, while a blue vehicle in the adjacent right lane follows behind and initiates a left turn.

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

EGO_INIT_DIST = [2, 8]
param EGO_SPEED = VerifaiRange(3, 6)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [12, 20]
param ADV_SPEED = VerifaiRange(3, 6)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 5
TERM_DIST = 60

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

# Ego (pink) is in the left lane of an approach that supports a U-turn
egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.U_TURN for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary (blue) is in an adjacent lane on the same approach, initiating a left turn
advInitLane = Uniform(*filter(lambda l: l.road is egoInitLane.road and l is not egoInitLane and any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers), intersection.incomingLanes))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST