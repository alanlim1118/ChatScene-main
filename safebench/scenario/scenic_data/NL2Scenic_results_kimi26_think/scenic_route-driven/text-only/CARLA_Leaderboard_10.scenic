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

param TRAFFIC_SPEED = VerifaiRange(15, 25)

DISTANCE_TO_JUNCTION1 = Uniform(20, 30) * -1

#################################
# AGENT BEHAVIORS               #
#################################

behavior TrafficBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.TRAFFIC_SPEED, trajectory=trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego exits via off-ramp (modeled as a right turn conflicting with straight traffic)
egoManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.RIGHT_TURN and m.intersection is not None, egoInitLane.maneuvers))

# Traffic goes straight on the highway (lane of moving traffic), conflicting with the off-ramp
straightManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
straightTrajectory = [straightManeuver.startLane, straightManeuver.connectingLane, straightManeuver.endLane]
trafficLane = straightManeuver.startLane

spwPt = trafficLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

traffic = new Car following roadDirection from spwPt for DISTANCE_TO_JUNCTION1,
    with blueprint MODEL,
    with behavior TrafficBehavior(trajectory = straightTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL
