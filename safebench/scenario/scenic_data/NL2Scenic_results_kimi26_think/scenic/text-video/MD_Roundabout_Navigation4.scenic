"""Scenario Description:

In a dark, nighttime environment, the ego vehicle navigates towards a multi-lane roundabout, following a lead vehicle with visible headlights. As the ego vehicle enters the junction to execute a right turn, the lead vehicle ahead abruptly halts within the roundabout. This sudden stop occurs because the lead vehicle must yield to three oncoming adversary vehicles entering the intersection from the upper-right lanes, their headlights becoming visible as they approach. Reacting to the lead vehicle's braking, the ego vehicle also comes to a complete stop directly behind it to prevent a collision, waiting for the traffic flow to clear.

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

EGO_INIT_DIST = [15, 30]
param EGO_SPEED = VerifaiRange(5, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

LEAD_INIT_DIST = [5, 15]
param LEAD_SPEED = VerifaiRange(5, 10)

ADV_INIT_DIST = [10, 25]
param ADV_SPEED = VerifaiRange(5, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
param FOLLOW_DIST = VerifaiRange(8, 15)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(trajectory, lead_vehicle):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate
    interrupt when (distance from self to lead_vehicle) < globalParameters.FOLLOW_DIST:
        take SetBrakeAction(globalParameters.EGO_BRAKE)
        do WaitBehavior()

behavior LeadBehavior(trajectory, adv1, adv2, adv3):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when (distance from self to adv1 < globalParameters.SAFETY_DIST) or (distance from self to adv2 < globalParameters.SAFETY_DIST) or (distance from self to adv3 < globalParameters.SAFETY_DIST):
        take SetBrakeAction(1.0)
        do WaitBehavior()

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable multi-lane intersection (roundabout)
intersection = Uniform(*filter(lambda i: len(i.incomingLanes) >= 3, network.intersections))

# Ego enters from one lane and takes a right turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle is ahead of ego on the same lane
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 20)

# Adversaries enter from a conflicting lane (upper-right relative to ego)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points for three adversaries along the conflicting lane
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint in advInitLane.centerline
advSpawnPt3 = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Dark nighttime environment
param time = 22
param weather = "ClearNight"

# Three adversary vehicles entering from upper-right
adv1 = new Car at advSpawnPt1,
    with blueprint MODEL,
    with behavior AdvBehavior(advTrajectory)

adv2 = new Car at advSpawnPt2,
    with blueprint MODEL,
    with behavior AdvBehavior(advTrajectory)

adv3 = new Car at advSpawnPt3,
    with blueprint MODEL,
    with behavior AdvBehavior(advTrajectory)

# Lead vehicle ahead of ego
lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(egoTrajectory, adv1, adv2, adv3)

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, lead)

# Requirements
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require LEAD_INIT_DIST[0] <= (distance from lead to intersection) <= LEAD_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv1 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adv3 to intersection) <= ADV_INIT_DIST[1]

# Ensure adversaries are spaced out approaching the intersection
require (distance from adv1 to intersection) < (distance from adv2 to intersection)
require (distance from adv2 to intersection) < (distance from adv3 to intersection)

terminate when (distance to egoSpawnPt) > TERM_DIST