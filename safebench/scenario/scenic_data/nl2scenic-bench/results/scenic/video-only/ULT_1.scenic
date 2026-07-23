"""Scenario Description:

The ego vehicle travels northbound straight through a four-way intersection, following a leading adversary that intends to turn left. The leading vehicle must yield to an opposing adversary traveling straight southbound. Consequently, the ego vehicle decelerates and stops behind the leading vehicle until the intersection clears.

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
LEAD_INIT_DIST = [12, 18]
OPP_INIT_DIST = [30, 45]

param EGO_SPEED = VerifaiRange(7, 10)
param LEAD_SPEED = VerifaiRange(6, 9)
param OPP_SPEED = VerifaiRange(7, 10)

param EGO_BRAKE = VerifaiRange(0.6, 1.0)
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
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Leading adversary is in same lane as ego, ahead, turning left
leadManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
leadTrajectory = [egoInitLane, leadManeuver.connectingLane, leadManeuver.endLane]
leadSpawnPt = new OrientedPoint in egoInitLane.centerline

# Opposing adversary comes from opposite direction going straight
oppManeuver = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers))
oppInitLane = oppManeuver.startLane
oppTrajectory = [oppInitLane, oppManeuver.connectingLane, oppManeuver.endLane]
oppSpawnPt = new OrientedPoint in oppInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

leadingAdversary = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=leadTrajectory)

opposingAdversary = new Car at oppSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPP_SPEED, trajectory=oppTrajectory)

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require LEAD_INIT_DIST[0] <= (distance from leadSpawnPt to intersection) <= LEAD_INIT_DIST[1]
require OPP_INIT_DIST[0] <= (distance from oppSpawnPt to intersection) <= OPP_INIT_DIST[1]
require (distance from egoSpawnPt to leadSpawnPt) > 5
terminate when (distance from ego to egoSpawnPt) > TERM_DIST