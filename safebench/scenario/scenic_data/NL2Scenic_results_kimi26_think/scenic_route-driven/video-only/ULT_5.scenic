"""Scenario Description:

In a low-light urban environment at night, the ego vehicle is positioned at a four-way intersection attempting to execute a left turn. The scene is dominated by the headlights of multiple adversary vehicles approaching from the opposing arm of the intersection. These oncoming vehicles proceed straight through the junction, their lights moving across the ego vehicle's field of view as they traverse the crossing. Because this stream of traffic directly intersects the ego vehicle's intended path, the ego vehicle is forced to yield, holding its position or moving slowly to allow the adversaries to pass. The maneuver requires careful timing to wait for a sufficient gap in the oncoming traffic flow before safely completing the left turn without causing a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

# Nighttime low-light conditions
param weather = 'ClearNight'

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [5, 15]

ADV1_INIT_DIST = [20, 30]
ADV2_INIT_DIST = [35, 50]
param ADV_SPEED = VerifaiRange(8, 12)

TERM_DIST = 60

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego: left turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversaries: straight through from the opposing arm
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL

adversary1 = new Car at advSpawnPt1,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary2 = new Car at advSpawnPt2,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Placement requirements
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV1_INIT_DIST[0] <= (distance from adversary1 to intersection) <= ADV1_INIT_DIST[1]
require ADV2_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV2_INIT_DIST[1]

terminate when (distance to egoSpawnPt) > TERM_DIST
