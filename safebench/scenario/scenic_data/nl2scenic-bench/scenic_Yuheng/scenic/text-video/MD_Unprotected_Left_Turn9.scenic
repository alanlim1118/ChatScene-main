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

EGO_INIT_DIST = [25, 35]
LEAD_INIT_DIST = [12, 18]
param EGO_SPEED = VerifaiRange(6, 9)
param LEAD_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(8, 12)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)
param LEAD_BRAKE = VerifaiRange(0.7, 1.0)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.LEAD_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

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

# Ego and lead share the same incoming lane and both perform left turns
egoInitLane = Uniform(*intersection.incomingLanes)
leftManeuvers = filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers)
egoManeuver = Uniform(*leftManeuvers)
leadManeuver = egoManeuver  # Same maneuver for lead vehicle

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
leadTrajectory = [egoInitLane, leadManeuver.connectingLane, leadManeuver.endLane]

# Adversary comes from the opposing arm and goes straight through
advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers)
    ).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points along centerlines
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leadSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather
param weather = Weather(preset='ClearNight')

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(leadTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Spatial constraints
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require LEAD_INIT_DIST[0] <= (distance from lead to ego) <= LEAD_INIT_DIST[1]
require lead is ahead of ego
require (distance from adversary to intersection) >= 15

terminate when (distance from ego to egoSpawnPt) > TERM_DIST