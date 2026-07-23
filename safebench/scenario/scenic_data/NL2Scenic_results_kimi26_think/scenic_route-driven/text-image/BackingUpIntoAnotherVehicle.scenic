"""Scenario Description:

In an urban area during daylight under clear weather conditions, a traffic scenario unfolds at a driveway or alley location with a posted speed limit of 25 mph where a vehicle is backing out. The diagram illustrates this vehicle reversing and turning from the side entrance into the main roadway, indicated by curved arrows near its rear bumper. Simultaneously, another vehicle is traveling straight down the adjacent lane, marked by a downward arrow. The path of the reversing vehicle intersects with the lane of the moving traffic, resulting in a collision between the backing car and the vehicle proceeding forward on the road.

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

EGO_INIT_DIST = [20, 40]

ADV_INIT_DIST = [5, 15]
param ADV_SPEED = VerifaiRange(2, 4)

TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and (m.intersection.is4Way or m.intersection.is3Way), egoInitLane.maneuvers))
intersection = egoManeuver.intersection

# Adversary vehicle emerges from a side road (driveway/alley) and turns into the main roadway,
# crossing the ego vehicle's path
advManeuver = Uniform(*filter(lambda m: m.type is not ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
