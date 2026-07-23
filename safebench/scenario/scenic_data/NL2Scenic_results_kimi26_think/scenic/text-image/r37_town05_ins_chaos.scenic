"""Scenario Description:

This top-down simulation view illustrates a complex urban traffic scenario at a four-way intersection flanked by city buildings with an overpass at the north end, a parking lot with numerous parked cars to the southwest, and a green park space to the southeast. The ego vehicle is positioned in the southern approach lane, executing a left turn into the intersection, while a leading vehicle directly ahead in the same lane initiates a right-hand turn. Cross traffic is present on the perpendicular roads, with two adversarial vehicles approaching from the west and two others moving from the right side toward the junction. Additionally, two oncoming vehicles are visible near the northern overpass, traveling south toward the intersection, and various colored trajectory lines overlay the road surface to indicate the diverse turning and straight paths of the agents navigating the scene.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_LEADING_SPEED = Range(2, 5)
param OPT_CROSS_SPEED = Range(4, 8)
param OPT_ONCOMING_SPEED = Range(4, 8)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior LeadingBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_LEADING_SPEED, leadingTrajectory)

behavior CrossBehavior(speed, traj):
    do FollowTrajectoryBehavior(speed, traj)

behavior OncomingBehavior(speed, traj):
    do FollowTrajectoryBehavior(speed, traj)

behavior ParkedBehavior():
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: left turn from southern approach
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Leading vehicle: right turn from the same approach road
egoIncomingRoad = egoInitLane.road
rightTurnManeuvers = list(filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.startLane.road is egoIncomingRoad, intersection.maneuvers))
require len(rightTurnManeuvers) >= 1
leadingManeuver = Uniform(*rightTurnManeuvers)
leadingInitLane = leadingManeuver.startLane
leadingTrajectory = [leadingInitLane, leadingManeuver.connectingLane, leadingManeuver.endLane]
leadingSpawnPt = new OrientedPoint in leadingInitLane.centerline

# Cross traffic and oncoming: use straight maneuvers from other approaches
straightManeuvers = list(filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane.road is not egoIncomingRoad, intersection.maneuvers))
require len(straightManeuvers) >= 3

crossManeuver1 = straightManeuvers[0]
crossManeuver2 = straightManeuvers[1]
oncomingManeuver = straightManeuvers[2]

cross1SpawnA = new OrientedPoint in crossManeuver1.startLane.centerline
cross1SpawnB = new OrientedPoint in crossManeuver1.startLane.centerline

cross2SpawnA = new OrientedPoint in crossManeuver2.startLane.centerline
cross2SpawnB = new OrientedPoint in crossManeuver2.startLane.centerline

oncomingSpawnA = new OrientedPoint in oncomingManeuver.startLane.centerline
oncomingSpawnB = new OrientedPoint in oncomingManeuver.startLane.centerline

# Parked cars to the southwest (approximate using offsets from ego approach)
parkedPt1 = new OrientedPoint left of egoSpawnPt by Range(8, 12), ahead of egoSpawnPt by -Range(15, 25)
parkedPt2 = new OrientedPoint left of egoSpawnPt by Range(8, 12), ahead of egoSpawnPt by -Range(30, 40)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Leading vehicle
leading = new Car at leadingSpawnPt,
    with heading leadingSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior LeadingBehavior()

# Cross traffic from west (first cross maneuver)
cross1A = new Car at cross1SpawnA,
    with heading cross1SpawnA.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CrossBehavior(globalParameters.OPT_CROSS_SPEED, [crossManeuver1.startLane, crossManeuver1.connectingLane, crossManeuver1.endLane])

cross1B = new Car at cross1SpawnB,
    with heading cross1SpawnB.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CrossBehavior(globalParameters.OPT_CROSS_SPEED, [crossManeuver1.startLane, crossManeuver1.connectingLane, crossManeuver1.endLane])

# Cross traffic from right side (second cross maneuver)
cross2A = new Car at cross2SpawnA,
    with heading cross2SpawnA.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CrossBehavior(globalParameters.OPT_CROSS_SPEED, [crossManeuver2.startLane, crossManeuver2.connectingLane, crossManeuver2.endLane])

cross2B = new Car at cross2SpawnB,
    with heading cross2SpawnB.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior CrossBehavior(globalParameters.OPT_CROSS_SPEED, [crossManeuver2.startLane, crossManeuver2.connectingLane, crossManeuver2.endLane])

# Oncoming traffic from north
oncomingA = new Car at oncomingSpawnA,
    with heading oncomingSpawnA.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED, [oncomingManeuver.startLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane])

oncomingB = new Car at oncomingSpawnB,
    with heading oncomingSpawnB.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior OncomingBehavior(globalParameters.OPT_ONCOMING_SPEED, [oncomingManeuver.startLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane])

# Parked cars in the lot to the southwest
parked1 = new Car at parkedPt1,
    with heading parkedPt1.heading,
    with regionContainedIn None,
    with behavior ParkedBehavior()

parked2 = new Car at parkedPt2,
    with heading parkedPt2.heading,
    with regionContainedIn None,
    with behavior ParkedBehavior()

# Placement constraints
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from leadingSpawnPt to intersection) <= 25
require (distance from egoSpawnPt to leadingSpawnPt) > 5

require 20 <= (distance from cross1SpawnA to intersection) <= 50
require 20 <= (distance from cross1SpawnB to intersection) <= 50
require (distance from cross1SpawnA to cross1SpawnB) > 10

require 20 <= (distance from cross2SpawnA to intersection) <= 50
require 20 <= (distance from cross2SpawnB to intersection) <= 50
require (distance from cross2SpawnA to cross2SpawnB) > 10

require 20 <= (distance from oncomingSpawnA to intersection) <= 50
require 20 <= (distance from oncomingSpawnB to intersection) <= 50
require (distance from oncomingSpawnA to oncomingSpawnB) > 10