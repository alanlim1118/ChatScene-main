"""Scenario Description:

This traffic scenario takes place at a T-junction with a two-way, two-lane road layout where a green vehicle under test (VUT) is performing a left turn. Two blue passenger cars are stationary in the lane, separated by a 1-meter gap due to traffic congestion. Simultaneously, a group of vulnerable road users consisting of a shared bicycle, a dog, and a pedestrian is crossing the road from left to right at a marked zebra crossing, with a spacing of 0.5 meters between each user. The turning path of the green vehicle intersects directly with the path of the pedestrian, leading to an estimated collision point where the front of the car would strike the pedestrian.

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
STATIONARY_CAR_MODEL = 'vehicle.tesla.model3'

param EGO_SPEED = VerifaiRange(3, 6)
EGO_BRAKE = 1.0

VRU_CROSS_SPEED = 1.2
VRU_THRESHOLD = 25

CRASH_DIST = 2.0
TERM_DIST = 60

STATIONARY_GAP = 1.0
VRU_SPACING = 0.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLeftTurnBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        take SetBrakeAction(EGO_BRAKE)
        terminate

behavior StationaryBehavior():
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        wait

behavior VRUCrossingBehavior(reference_actor, speed, threshold):
    do CrossingBehavior(reference_actor, speed, threshold)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction (3-way intersection)
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego performs a left turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Define spawn points for stationary cars in the target/end lane of the left turn
endLane = egoManeuver.endLane
stationaryBasePt = new OrientedPoint in endLane.centerline

# VRU crossing reference point: on the left side of the end lane, crossing left-to-right
# relative to the end lane heading
crossingRefPt = new OrientedPoint at endLane.leftEdge.start,
    with heading endLane.centerline.end.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green Vehicle Under Test (Ego)
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 1, 0),
    with behavior EgoLeftTurnBehavior(egoTrajectory),
    with regionContainedIn None

# Two stationary blue passenger cars in the end lane with 1m gap
stationaryCar1 = new Car ahead of stationaryBasePt by 5,
    with blueprint STATIONARY_CAR_MODEL,
    with color (0, 0, 1),
    with behavior StationaryBehavior(),
    with regionContainedIn None

stationaryCar2 = new Car ahead of stationaryCar1 by (4.5 + STATIONARY_GAP),
    with blueprint STATIONARY_CAR_MODEL,
    with color (0, 0, 1),
    with behavior StationaryBehavior(),
    with regionContainedIn None

# Vulnerable Road Users crossing from left to right at zebra crossing
# Pedestrian is the primary collision target
pedestrian = new Pedestrian left of crossingRefPt by 3,
    with heading crossingRefPt.heading + 90 deg,
    with regionContainedIn None,
    with behavior VRUCrossingBehavior(ego, VRU_CROSS_SPEED, VRU_THRESHOLD)

# Shared bicycle positioned 0.5m behind pedestrian in crossing direction
bicycle = new Pedestrian behind pedestrian by VRU_SPACING,
    with heading pedestrian.heading,
    with regionContainedIn None,
    with behavior VRUCrossingBehavior(ego, VRU_CROSS_SPEED, VRU_THRESHOLD)

# Dog positioned 0.5m behind bicycle in crossing direction
dog = new Pedestrian behind bicycle by VRU_SPACING,
    with heading pedestrian.heading,
    with regionContainedIn None,
    with behavior VRUCrossingBehavior(ego, VRU_CROSS_SPEED, VRU_THRESHOLD)

# Constraints
require 15 <= (distance from ego to intersection) <= 30
require pedestrian in network.drivableRegion or True  # Allow VRUs outside drivable region
terminate when (distance from ego to egoSpawnPt) > TERM_DIST