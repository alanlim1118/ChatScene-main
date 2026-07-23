"""Scenario Description:

The ego-vehicle must cross a lane of moving traffic to exit the highway at an off-ramp.

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

param EGO_SPEED = VerifaiRange(15, 25)
param TRAFFIC_SPEED = VerifaiRange(15, 25)
param SAFETY_DIST = VerifaiRange(10, 20)
BRAKE_INTENSITY = 1.0

DISTANCE_TO_JUNCTION1 = Uniform(20, 30) * -1
DISTANCE_TO_JUNCTION2 = Uniform(20, 30) * -1

#################################
# AGENT BEHAVIORS               #
#################################

behavior TrafficBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.TRAFFIC_SPEED, trajectory=trajectory)
    terminate

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(BRAKE_INTENSITY)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a highway junction (off-ramp)
junction = Uniform(*filter(lambda i: not i.is4Way, network.intersections))

# Traffic goes straight on the highway (lane of moving traffic)
trafficLane = Uniform(*junction.incomingLanes)
straightManeuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, trafficLane.maneuvers)
straightManeuver = Uniform(*straightManeuvers)
straightTrajectory = [straightManeuver.startLane, straightManeuver.connectingLane, straightManeuver.endLane]

# Ego exits via off-ramp (modeled as a right turn conflicting with straight traffic)
egoManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.RIGHT_TURN, straightManeuver.conflictingManeuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

spwPt = trafficLane.centerline[-1]
egoSpwPt = egoManeuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

traffic = new Car following roadDirection from spwPt for DISTANCE_TO_JUNCTION1,
    with blueprint MODEL,
    with behavior TrafficBehavior(trajectory = straightTrajectory)

ego = new Car following roadDirection from egoSpwPt for DISTANCE_TO_JUNCTION2,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)