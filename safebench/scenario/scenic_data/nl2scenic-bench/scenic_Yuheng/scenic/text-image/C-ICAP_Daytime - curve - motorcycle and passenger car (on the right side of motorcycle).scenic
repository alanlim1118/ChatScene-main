"""Scenario Description:

The scenario illustrates a road test environment featuring a straight section merging into a right-hand curve with a 500-meter radius, marked by white dotted lane lines across at least three lanes. A green vehicle travels along the straight section in the left lane, approaching the curve where a stationary motorcycle is positioned on the lane center line. To the right of the motorcycle, a stationary blue passenger car is stopped with its left wheels situated 0.2 meters from the lane line, presenting obstacles within the curve that is designed to accommodate at least 5 seconds of driving time.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
GREEN_CAR_MODEL = "vehicle.tesla.model3"
BLUE_CAR_MODEL = "vehicle.audi.a2"
MOTORCYCLE_MODEL = "vehicle.kawasaki.ninja"

param OPT_EGO_SPEED = Range(8, 12)  # Speed to ensure >= 5 seconds of driving
CURVE_RADIUS = 500
LANE_OFFSET_FROM_LINE = 0.2
MIN_DRIVING_TIME = 5

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoFollowLaneBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (withinDistanceToObjsInLane(self, 15)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable road segment: straight section leading into a curve with >= 3 lanes
# We search for lanes that have a successor forming a right-hand curve
candidateSections = []
for section in network.laneSections:
    if section.lanes and len(section.lanes) >= 3:
        leftLane = section.lanes[0]
        if leftLane.successor is not None:
            candidateSections.append(section)

require len(candidateSections) > 0
chosenSection = Uniform(*candidateSections)

# Left lane for the green vehicle (ego's lane)
leftLane = chosenSection.lanes[0]
# Middle lane where obstacles are placed
middleLane = chosenSection.lanes[1] if len(chosenSection.lanes) > 1 else chosenSection.lanes[0]

# Ensure we have enough straight road before the curve for approach
straightSegment = leftLane.centerline
curveStart = leftLane.successor.centerline.start if leftLane.successor else leftLane.centerline.end

# Place ego spawn point on the straight section, sufficiently back from curve
# to allow at least 5 seconds of driving at target speed
minApproachDist = globalParameters.OPT_EGO_SPEED * MIN_DRIVING_TIME
egoSpawnPt = new OrientedPoint on straightSegment,
    with distanceTo curveStart >= minApproachDist

# Motorcycle positioned on the middle lane centerline near the curve entrance
motorcycleBasePt = new OrientedPoint on middleLane.centerline,
    with distanceTo curveStart <= 30

# Blue car to the right of motorcycle, with left wheels 0.2m from lane line
# The lane line between middle and right lane; offset toward right lane
laneLineWidth = 0.15  # approximate lane marking width
blueCarOffset = LANE_OFFSET_FROM_LINE + laneLineWidth / 2.0
blueCarBasePt = new OrientedPoint at motorcycleBasePt.position offset laterally by blueCarOffset,
    with heading motorcycleBasePt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green vehicle (adversarial/lead vehicle) traveling in left lane
greenVehicle = new Car at egoSpawnPt,
    with blueprint GREEN_CAR_MODEL,
    with color (0, 1, 0),
    with regionContainedIn None,
    with behavior EgoFollowLaneBehavior(globalParameters.OPT_EGO_SPEED)

# Stationary motorcycle on middle lane centerline
stationaryMotorcycle = new Motorcycle at motorcycleBasePt,
    with blueprint MOTORCYCLE_MODEL,
    with heading motorcycleBasePt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior()

# Stationary blue passenger car to the right of motorcycle
blueCar = new Car at blueCarBasePt,
    with blueprint BLUE_CAR_MODEL,
    with color (0, 0, 1),
    with heading blueCarBasePt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior()

# Require that the curve exists and has appropriate geometry
require leftLane.successor is not None
require distance from motorcycleBasePt to curveStart <= 30
require distance from blueCarBasePt to curveStart <= 35