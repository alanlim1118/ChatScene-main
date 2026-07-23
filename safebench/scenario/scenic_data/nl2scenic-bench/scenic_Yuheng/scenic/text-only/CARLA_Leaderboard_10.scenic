"""Scenario Description:

The ego-vehicle must cross a lane of moving traffic to exit the highway at an off-ramp.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [30, 50]
param EGO_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

TRAFFIC_INIT_DIST = [20, 40]
param TRAFFIC_SPEED = VerifaiRange(10, 15)

param SAFETY_DIST = VerifaiRange(12, 22)
CRASH_DIST = 5
TERM_DIST = 120

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

# Find a highway section with an off-ramp (right turn maneuver from a straight lane)
highwayLanes = filter(lambda l: len(l.maneuvers) > 0, network.lanes)
exitManeuvers = filter(lambda m: m.type is ManeuverType.RIGHT_TURN, 
                       [m for l in highwayLanes for m in l.maneuvers])
exitManeuver = Uniform(*exitManeuvers)

egoStartLane = exitManeuver.startLane
egoTrajectory = [egoStartLane, exitManeuver.connectingLane, exitManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoStartLane.centerline

# Traffic vehicle continues straight on the highway (same lane or adjacent through lane)
straightManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStartLane.maneuvers)
require len(straightManeuvers) > 0
trafficManeuver = Uniform(*straightManeuvers)
trafficTrajectory = [trafficManeuver.startLane, trafficManeuver.connectingLane, trafficManeuver.endLane]
trafficSpawnPt = new OrientedPoint in trafficManeuver.startLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

traffic_car = new Car at trafficSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.TRAFFIC_SPEED, trajectory=trafficTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to exitManeuver.connectingLane) <= EGO_INIT_DIST[1]
require TRAFFIC_INIT_DIST[0] <= (distance from traffic_car to exitManeuver.connectingLane) <= TRAFFIC_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST