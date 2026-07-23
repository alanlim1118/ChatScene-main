"""Scenario Description:

The ego vehicle travels straight along a paved urban road approaching a T-junction under clear, sunny conditions, maintaining a steady pace. Prior to reaching the intersection, a pedestrian crosses the street from the left side, clearing the path before the ego vehicle arrives. As the ego vehicle enters and passes through the junction, a grey vehicle waiting on the left side road executes a left turn, merging onto the main road behind the ego vehicle. The ego vehicle continues its forward trajectory past residential and commercial buildings, while the merging vehicle aligns itself in the lane behind, and other parked vehicles are visible in a lot to the left.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MERGING_VEHICLE_MODEL = "vehicle.tesla.model3"
PARKED_CAR_MODEL = "vehicle.nissan.patrol"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_PED_SPEED = Range(1.5, 2.5)
param OPT_MERGE_DELAY = Range(1, 3)
param OPT_BRAKE_DIST = Range(6, 10)

PEDESTRIAN_CLEAR_DISTANCE = 5
MERGE_TRIGGER_DISTANCE = 15
NUM_PARKED_CARS = 4

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoStraightBehavior():
    try:
        do FollowLaneBehavior(globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for 3 seconds
        abort
    terminate

behavior PedestrianCrossAndClearBehavior(ego_ref, cross_speed):
    # Wait until ego is close enough that crossing will complete before arrival
    while distance from self to ego_ref > 30:
        wait
    take SetWalkingSpeedAction(cross_speed)
    # Cross perpendicular to road heading (from left to right relative to ego)
    take SetWalkingDirectionAction(self.heading)
    # Stop after clearing the ego's path
    while distance from self to ego_ref.lane.centerline > PEDESTRIAN_CLEAR_DISTANCE:
        wait
    take SetWalkingSpeedAction(0)
    do WaitBehavior()

behavior MergeAfterEgoBehavior(ego_ref, merge_delay):
    # Wait at stop position until ego has passed through intersection
    while distance from ego_ref to self > MERGE_TRIGGER_DISTANCE or ego_ref.speed < 1:
        wait
    # Additional delay to ensure ego is well past
    do WaitBehavior() for merge_delay seconds
    # Execute left turn onto main road following ego
    do FollowLaneBehavior(Range(6, 10))

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction (3-way intersection)
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego approaches straight through the T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, 
                               [m for lane in intersection.incomingLanes for m in lane.maneuvers]))
egoInitLane = egoManeuver.startLane
egoEndLane = egoManeuver.endLane
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoEndLane.centerline

# Spawn ego upstream of intersection
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Identify the left incoming lane at the T-junction for the merging vehicle
leftIncomingLanes = filter(lambda l: abs(relativeAngle(l.orientation, egoInitLane.orientation)) > 45 deg,
                           intersection.incomingLanes)
mergeStartLane = Uniform(*leftIncomingLanes) if leftIncomingLanes else egoInitLane
mergeManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, mergeStartLane.maneuvers))

# Merging vehicle spawn point on the left side road
mergeSpawnPt = new OrientedPoint in mergeStartLane.centerline

# Pedestrian spawns on the left sidewalk/crosswalk area ahead of ego but before intersection
pedCrossingPoint = new OrientedPoint on egoInitLane.leftEdge,
    with heading egoInitLane.orientation + 90 deg  # Facing right across the road (from left side)
pedSpawnRegion = RectangularRegion(pedCrossingPoint.position, pedCrossingPoint.heading, 3, 2)

# Parking lot region to the left of the end lane
parkingLotBase = new OrientedPoint on egoEndLane.leftEdge,
    with heading egoEndLane.orientation + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather: clear and sunny
param weather = WeatherConditions(precipitation=0, cloudiness=0, sunAltitude=70)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoStraightBehavior()

# Pedestrian crossing from left, clearing before ego arrives
pedestrian = new Pedestrian in pedSpawnRegion,
    with heading pedCrossingPoint.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossAndClearBehavior(ego, globalParameters.OPT_PED_SPEED)

# Merging grey vehicle on left side road
mergingVehicle = new Car at mergeSpawnPt,
    with regionContainedIn None,
    with blueprint MERGING_VEHICLE_MODEL,
    with color Vector(0.5, 0.5, 0.5),
    with behavior MergeAfterEgoBehavior(ego, globalParameters.OPT_MERGE_DELAY)

# Parked vehicles in lot to the left of the end lane
for i in range(NUM_PARKED_CARS):
    offsetDist = 8 + (i * 5)
    parkSpot = new OrientedPoint left of parkingLotBase by offsetDist,
        with heading parkingLotBase.heading
    new Car at parkSpot,
        with regionContainedIn None,
        with blueprint PARKED_CAR_MODEL,
        with throttle 0,
        with brake 1

# Ensure ego starts at reasonable distance from intersection
require 40 <= (distance from ego to intersection) <= 70

# Ensure merging vehicle is on a distinct lane from ego
require mergeStartLane is not egoInitLane

# Ensure pedestrian is between ego and intersection
require distance from ego to pedestrian < distance from ego to intersection

terminate after 45 seconds