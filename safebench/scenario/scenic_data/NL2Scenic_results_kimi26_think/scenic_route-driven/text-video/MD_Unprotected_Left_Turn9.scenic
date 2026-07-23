"""Scenario Description:

Under dark nighttime conditions, the ego vehicle travels through a multi-lane, four-way urban intersection, following a front vehicle that is also preparing to execute a left turn. As the vehicles navigate the junction, the front vehicle brakes to yield the right-of-way to oncoming adversary vehicles passing straight through from the opposing arm. In response to the lead vehicle's deceleration and the presence of the oncoming traffic, the ego vehicle also applies its brakes to maintain a safe following distance and avoid a collision while waiting for the intersection to clear.

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

# Lead vehicle (front)
LEAD_INIT_DIST = [5, 10]
param LEAD_SPEED = VerifaiRange(5, 8)
param LEAD_BRAKE = VerifaiRange(0.5, 1.0)

# Ego vehicle
EGO_INIT_DIST = [25, 35]

# Adversary (oncoming straight)
ADV_INIT_DIST = [15, 25]
param ADV_SPEED = VerifaiRange(7, 10)

# Following and safety distances
param YIELD_DIST = VerifaiRange(15, 25)
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(trajectory, adversary):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when (distance from self to adversary) < globalParameters.YIELD_DIST:
        take SetBrakeAction(globalParameters.LEAD_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary comes from the opposing arm, going straight through the intersection
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points
leadSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather
param weather = 'ClearNight'

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(egoTrajectory, adversary)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

# Position constraints
require LEAD_INIT_DIST[0] <= (distance from lead to intersection) <= LEAD_INIT_DIST[1]
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

# Ensure ego is behind lead
require (distance from ego to intersection) > (distance from lead to intersection)

terminate when (distance to egoSpawnPt) > TERM_DIST
