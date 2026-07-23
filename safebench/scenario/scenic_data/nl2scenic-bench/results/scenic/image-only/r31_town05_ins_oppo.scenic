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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
LEAD_MODEL = 'vehicle.tesla.model3'
ADV_MODEL = 'vehicle.audi.tt'

param EGO_SPEED = VerifaiRange(6, 9)
param LEAD_SPEED = VerifaiRange(6, 9)
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
TERM_DIST = 80
LEAD_FOLLOW_DIST = Range(12, 18)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(1.0)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior LeadBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection; prefer signalized if available for realistic urban setting
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego performs a LEFT TURN maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle follows same left-turn trajectory, spawned ahead of ego
leadSpawnPt = new OrientedPoint ahead of egoSpawnPt by LEAD_FOLLOW_DIST,
    with heading egoSpawnPt.heading

# Adversary goes STRAIGHT from the opposing (northern) arm
# Find the reverse straight maneuver relative to ego's start lane to get the opposing lane
opposingStraightManeuvers = filter(lambda m:
    m.type is ManeuverType.STRAIGHT,
    egoManeuver.reverseManeuvers
)
advManeuver = Uniform(*opposingStraightManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory),
    with color "0,0,255"

leadVehicle = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadBehavior(egoTrajectory),
    with color "0,100,255"

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory),
    with color "255,200,0"

# Ensure proper initial distances from intersection
require 20 <= (distance from egoSpawnPt to intersection) <= 35
require 20 <= (distance from advSpawnPt to intersection) <= 40

# Ensure lead is ahead of ego in the same lane
require (distance from egoSpawnPt to leadSpawnPt) <= LEAD_FOLLOW_DIST[1]

# Terminate after sufficient travel distance
terminate when (distance from ego to egoSpawnPt) > TERM_DIST