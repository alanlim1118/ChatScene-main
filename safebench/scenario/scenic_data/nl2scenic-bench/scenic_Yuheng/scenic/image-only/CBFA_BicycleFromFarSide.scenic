"""Scenario Description:

This diagram illustrates a vehicle safety test scenario in which a white Vehicle under Test travels forward along axis BB towards a bicyclist crossing its path from the farside along trajectory AA. Two stationary black vehicles are positioned to the left of the bicyclist's path. The bicyclist accelerates over a distance N, which is less than 8 meters and outside the vehicle's field of view, before traveling a steady state distance O of 22.00 meters to reach point Q. Point Q marks the impact position for a 50% farside scenario, where the frontal structure of the white vehicle strikes the bicyclist without any braking action applied, while the bicyclist travels on a path laterally offset by 1.00 meter from the reference line of the stationary vehicles.

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
STATIONARY_VEHICLE_MODEL = "vehicle.tesla.model3"

# Bicyclist parameters
ACCEL_DISTANCE_N = Uniform(4, 7)  # Less than 8 meters
STEADY_STATE_DISTANCE_O = 22.00
LATERAL_OFFSET = 1.00

# Speeds
EGO_SPEED = Range(8, 12)  # m/s, typical urban speed
BICYCLE_CRUISING_SPEED = Range(3, 5)  # m/s, realistic bicycle speed

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoNoBrakeBehavior(speed, trajectory):
    """Ego vehicle follows trajectory without any braking."""
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    terminate

behavior BicycleAccelerateThenCruiseBehavior(accel_distance, cruise_speed, total_steady_distance):
    """Bicyclist accelerates over accel_distance then maintains cruise_speed."""
    # Acceleration phase: approximate with lower speed then transition
    accel_speed = cruise_speed * 0.5
    do CrossingBehavior(ego, accel_speed, accel_distance)
    # Steady state cruising phase for remaining distance
    remaining = total_steady_distance - accel_distance
    if remaining > 0:
        do CrossingBehavior(ego, cruise_speed, remaining)
    terminate

behavior StationaryBehavior():
    """Vehicle remains completely stationary."""
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select an intersection and maneuver for the ego vehicle
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoEndLane = egoManeuver.endLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoEndLane]

# Ego spawn point on the start lane centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Define the crossing reference line on the end lane (farside)
crossingRefPt = new OrientedPoint in egoEndLane.centerline,
    facing roadDirection

# Bicyclist crosses from farside, laterally offset by LATERAL_OFFSET from reference
# The bicyclist path is perpendicular to ego direction, offset from stationary vehicle reference
bicycleCrossingOrigin = new OrientedPoint at crossingRefPt offset by LATERAL_OFFSET @ -90 deg,
    facing crossingRefPt.heading + 90 deg  # Perpendicular crossing direction

# Two stationary vehicles positioned to the left of the bicyclist's path
# "Left" relative to bicyclist crossing direction means negative lateral offset
stationaryOffset1 = LATERAL_OFFSET + 2.5  # Further left from bicycle path
stationaryOffset2 = LATERAL_OFFSET + 5.5

stationaryPt1 = new OrientedPoint at crossingRefPt offset by stationaryOffset1 @ -90 deg,
    facing crossingRefPt.heading
stationaryPt2 = new OrientedPoint at crossingRefPt offset by stationaryOffset2 @ -90 deg,
    facing crossingRefPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# White Vehicle Under Test - no braking behavior
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "255,255,255",
    with regionContainedIn None,
    with behavior EgoNoBrakeBehavior(EGO_SPEED, egoTrajectory)

# Bicyclist crossing from farside with acceleration then steady state
bicyclist = new Bicycle at bicycleCrossingOrigin,
    with heading bicycleCrossingOrigin.heading,
    with regionContainedIn None,
    with behavior BicycleAccelerateThenCruiseBehavior(ACCEL_DISTANCE_N, BICYCLE_CRUISING_SPEED, STEADY_STATE_DISTANCE_O)

# Two stationary black vehicles to the left of bicyclist path
stationaryCar1 = new Car at stationaryPt1,
    with blueprint STATIONARY_VEHICLE_MODEL,
    with color "0,0,0",
    with regionContainedIn None,
    with behavior StationaryBehavior()

stationaryCar2 = new Car at stationaryPt2,
    with blueprint STATIONARY_VEHICLE_MODEL,
    with color "0,0,0",
    with regionContainedIn None,
    with behavior StationaryBehavior()

# Ensure reasonable distances for the scenario setup
require 30 <= (distance from egoSpawnPt to intersection) <= 60
require (distance from bicycleCrossingOrigin to egoEndLane.centerline) < 5

# Terminate after sufficient simulation time or when ego passes the intersection
terminate when (distance from ego to crossingRefPt) > 40 or simulation().currentTime > 30