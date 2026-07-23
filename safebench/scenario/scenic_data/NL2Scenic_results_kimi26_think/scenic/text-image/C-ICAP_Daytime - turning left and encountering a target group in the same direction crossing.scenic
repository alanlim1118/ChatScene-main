"""Scenario Description:

This traffic scenario takes place at a T-junction with a two-way, two-lane road layout where a green vehicle under test (VUT) is performing a left turn. Two blue passenger cars are stationary in the lane, separated by a 1-meter gap due to traffic congestion. Simultaneously, a group of vulnerable road users consisting of a shared bicycle, a dog, and a pedestrian is crossing the road from left to right at a marked zebra crossing, with a spacing of 0.5 meters between each user. The turning path of the green vehicle intersects directly with the path of the pedestrian, leading to an estimated collision point where the front of the car would strike the pedestrian. Distance markers of 1 meter are visible near the curb and between the stationary vehicles.

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
BLUE_CAR_MODEL = 'vehicle.audi.a2'
BICYCLE_MODEL = 'vehicle.bh.crossbike'

param EGO_SPEED = Range(5, 8)
EGO_BRAKE = 1.0

PED_SPEED = 1.0
BIKE_SPEED = 0.3

SAFETY_DIST = 8
CRASH_DIST = 3
TERM_DIST = 60

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
        take SetBrakeAction(EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior StationaryBehavior():
    while True:
        wait

behavior WalkAcross(speed):
    take SetWalkingSpeedAction(speed)
    while True:
        wait

behavior DriveAcross(speed):
    while True:
        take SetThrottleAction(speed)

#################################
# SPATIAL RELATIONS             #
#################################

# T-junction (3-way intersection)
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego left-turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# End lane where the crosswalk and stationary vehicles are located
endLane = egoManeuver.endLane
endLaneHeading = endLane.centerline.start.heading

# Stationary blue cars in the end lane with ~1 meter gap between them
# Assuming car length ~4.5 m, center-to-center offset ~5.5 m yields ~1 m bumper gap
car1Pt = new OrientedPoint at endLane.centerline.start offset by (8 @ endLaneHeading)
car2Pt = new OrientedPoint at car1Pt offset by (-5.5 @ endLaneHeading)

# Crosswalk reference point near the intersection on the left side of the end lane
crossRef = new OrientedPoint at endLane.leftEdge.start offset by (2 @ endLaneHeading)
crossHeading = endLaneHeading + 90 deg

# VRUs spaced by 0.5 m along the road direction (parallel to end lane)
# Order: pedestrian at estimated collision point, then dog, then bicycle
pedPt = new OrientedPoint at crossRef
dogPt = new OrientedPoint at pedPt offset by (0.5 @ endLaneHeading)
bikePt = new OrientedPoint at dogPt offset by (0.5 @ endLaneHeading)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (green VUT)
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color Color(0, 1, 0),
    with behavior EgoBehavior(egoTrajectory)

# Stationary blue passenger cars
car1 = new Car at car1Pt,
    with blueprint BLUE_CAR_MODEL,
    with color Color(0, 0, 1),
    with behavior StationaryBehavior()

car2 = new Car at car2Pt,
    with blueprint BLUE_CAR_MODEL,
    with color Color(0, 0, 1),
    with behavior StationaryBehavior()

# Vulnerable road users crossing from left to right at the zebra crossing
ped = new Pedestrian at pedPt,
    with heading crossHeading,
    with regionContainedIn None,
    with behavior WalkAcross(PED_SPEED)

dog = new Pedestrian at dogPt,
    with heading crossHeading,
    with regionContainedIn None,
    with behavior WalkAcross(PED_SPEED)

bike = new Bicycle at bikePt,
    with heading crossHeading,
    with blueprint BICYCLE_MODEL,
    with regionContainedIn None,
    with behavior DriveAcross(BIKE_SPEED)

# Requirements
require 20 <= (distance to intersection) <= 30
terminate when (distance to egoSpawnPt) > TERM_DIST