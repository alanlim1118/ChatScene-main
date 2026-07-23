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

TRAFFIC_INIT_DIST = [20, 40]
param TRAFFIC_SPEED = VerifaiRange(10, 15)

TERM_DIST = 120

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoStartLane = network.laneAt(egoSpawnPt.position)

# Find the off-ramp (right turn maneuver) from the ego's own lane
exitManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoStartLane.maneuvers))

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
    with blueprint MODEL

traffic_car = new Car at trafficSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.TRAFFIC_SPEED, trajectory=trafficTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to exitManeuver.connectingLane) <= EGO_INIT_DIST[1]
require TRAFFIC_INIT_DIST[0] <= (distance from traffic_car to exitManeuver.connectingLane) <= TRAFFIC_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST
