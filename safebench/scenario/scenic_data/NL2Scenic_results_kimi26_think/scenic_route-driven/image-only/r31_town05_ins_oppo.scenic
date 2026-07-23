"""Scenario Description:

An aerial view captures a traffic scenario at a four-way urban intersection where the ego vehicle is following a blue lead vehicle in the southern approach lane, preparing to turn left as indicated by a blue trajectory arc. As the ego vehicle and its lead initiate the left turn across the intersection, a yellow adversary vehicle approaches from the opposing northern arm and proceeds straight through the junction. This straight path of the oncoming yellow vehicle creates a direct conflict with the turning trajectory of the ego vehicle. The intersection is surrounded by commercial buildings to the north, a parking lot with parked cars and stacked colorful containers to the southwest, and a green park area to the southeast.

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
LEAD_INIT_DIST = [10, 20]
ADV_INIT_DIST = [25, 40]

param LEAD_SPEED = Range(6, 9)
param ADV_SPEED = Range(8, 12)

TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior FollowTrajectory(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

# Ego and lead: left turn from southern approach
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary: straight from opposing northern arm (conflicts with ego left turn)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points on centerlines
leadSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Lead vehicle (blue) ahead of ego, turning left
lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with color Color(0, 0, 1),
    with behavior FollowTrajectory(egoTrajectory, globalParameters.LEAD_SPEED)

# Ego vehicle following lead, turning left
ego = new Car at egoSpawnPt,
    with blueprint MODEL

# Adversary vehicle (yellow) approaching from opposite direction, going straight
adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color Color(1, 1, 0),
    with behavior FollowTrajectory(advTrajectory, globalParameters.ADV_SPEED)

#################################
# REQUIREMENTS                  #
#################################

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require LEAD_INIT_DIST[0] <= (distance from leadSpawnPt to intersection) <= LEAD_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]

# Ensure lead is ahead of ego (closer to intersection)
require (distance from leadSpawnPt to intersection) < (distance from egoSpawnPt to intersection)

terminate when (distance to egoSpawnPt) > TERM_DIST
