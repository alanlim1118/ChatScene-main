"""Scenario Description:

Figure 33 presents a top-down visualization of a "Move Out of Travel Lane/Park Test Scenario" on a straight urban street divided by a dashed white line. An autonomous driving system (ADS) equipped vehicle, highlighted with a green outline, is traveling in the left lane and needs to exit the active travel lane. To its right, two stationary white vehicles are parked in a line, creating a gap labeled as the "Desired Parking Location." A curved arrow illustrates the intended path of the green vehicle as it maneuvers from the travel lane into this specific parking spot between the stationary cars, fulfilling the objective of moving out of traffic to allow for passenger embarkation or disembarkation.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
PARKED_CAR_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_PARKING_SPEED = Range(1, 3)
param OPT_GAP_LENGTH = Range(6, 8)  # Length of the desired parking gap
param OPT_PARKED_CAR_OFFSET = Range(1.5, 2.5)  # Lateral offset from lane center to parked position
param OPT_EGO_START_DISTANCE = Range(30, 50)  # Distance before the gap where ego starts maneuvering
param OPT_BRAKE_DIST = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior ParkInGapBehavior(target_point, approach_speed, park_speed):
    # Drive forward in lane until close to the parking gap
    do FollowLaneBehavior(target_speed=approach_speed)
    interrupt when (distance from self to target_point < 15):
        take SetThrottleAction(0)
        take SetBrakeAction(0.5)
        do WaitBehavior() for 1 seconds
        # Maneuver toward the parking spot
        do DriveToTargetBehavior(target_point, target_speed=park_speed)
        take SetBrakeAction(1)
        do WaitBehavior() for 10 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment suitable for parking
straightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and len(m.startLane.centerline.points) > 20, network.maneuvers))
egoLane = straightManeuver.startLane

# Define reference points along the lane
laneCenterline = egoLane.centerline
gapCenterPt = new OrientedPoint in laneCenterline
    with require distance from gapCenterPt to laneCenterline.start > 40,
    with require distance from gapCenterPt to laneCenterline.end > 40

# Points for parked cars relative to the gap center
parkedCarFrontRef = new OrientedPoint following egoLane.orientation from gapCenterPt for globalParameters.OPT_GAP_LENGTH / 2 + 3
parkedCarRearRef = new OrientedPoint following egoLane.orientation from gapCenterPt for -(globalParameters.OPT_GAP_LENGTH / 2 + 3)

# Actual parked car positions (offset to the right side of the lane)
parkedCarFrontPos = new OrientedPoint right of parkedCarFrontRef by globalParameters.OPT_PARKED_CAR_OFFSET,
    with heading parkedCarFrontRef.heading
parkedCarRearPos = new OrientedPoint right of parkedCarRearRef by globalParameters.OPT_PARKED_CAR_OFFSET,
    with heading parkedCarRearRef.heading

# Target parking position in the gap
parkingTarget = new OrientedPoint right of gapCenterPt by globalParameters.OPT_PARKED_CAR_OFFSET,
    with heading gapCenterPt.heading

# Ego spawn point upstream of the gap
egoSpawnPt = new OrientedPoint following egoLane.orientation from gapCenterPt for -globalParameters.OPT_EGO_START_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with color (0, 255, 0),  # Green outline as specified
    with behavior ParkInGapBehavior(parkingTarget, globalParameters.OPT_EGO_SPEED, globalParameters.OPT_PARKING_SPEED)

ParkedCarFront = new Car at parkedCarFrontPos,
    with blueprint PARKED_CAR_MODEL,
    with regionContainedIn None,
    with color (255, 255, 255),  # White stationary vehicle
    with behavior WaitBehavior()

ParkedCarRear = new Car at parkedCarRearPos,
    with blueprint PARKED_CAR_MODEL,
    with regionContainedIn None,
    with color (255, 255, 255),  # White stationary vehicle
    with behavior WaitBehavior()

# Ensure the gap is clear and properly sized
require distance from ParkedCarFront to ParkedCarRear >= globalParameters.OPT_GAP_LENGTH
require distance from egoSpawnPt to gapCenterPt >= 20

terminate when ego is visible from parkingTarget and distance from ego to parkingTarget < 1.0