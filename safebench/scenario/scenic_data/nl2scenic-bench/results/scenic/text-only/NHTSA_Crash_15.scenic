"""Scenario Description:

A northbound vehicle, A, was waiting to make a left turn. The light changed and the northbound vehicle began to turn left. A southbound driver, B, accelerated hard, hoping to make the light and struck Vehicle A.

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

# Vehicle A (left-turning) parameters
param OPT_A_SPEED = Range(3, 6)          # Moderate speed for left turn
param OPT_A_START_DIST = Range(25, 35)   # Distance from intersection when starting turn

# Vehicle B (southbound speeding) parameters
param OPT_B_SPEED = Range(12, 18)        # High speed - accelerating hard to beat light
param OPT_B_START_DIST = Range(30, 45)   # Distance from intersection when scenario starts

#################################
# MONITORS                      #
#################################

monitor TrafficLightController():
    """Control traffic lights to enable the scenario:
    - Initially red for northbound (Vehicle A waits)
    - Then green for northbound (Vehicle A turns)
    - Yellow/red for southbound but Vehicle B runs it
    """
    freezeTrafficLights()
    while True:
        # Set northbound light to green so Vehicle A can begin turning
        if withinDistanceToTrafficLight(VehicleA, 80):
            setClosestTrafficLightStatus(VehicleA, "green")
        # Southbound light should be yellow or red, but Vehicle B accelerates anyway
        if withinDistanceToTrafficLight(VehicleB, 80):
            setClosestTrafficLightStatus(VehicleB, "yellow")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeftTurnBehavior(trajectory, target_speed):
    """Vehicle A performs a left turn following the maneuver trajectory."""
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(car=self, thresholdDistance=5):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

behavior AccelerateThroughIntersectionBehavior(trajectory, target_speed):
    """Vehicle B accelerates hard through the intersection, hoping to beat the light."""
    brakeIntensity = 0.9
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(car=self, thresholdDistance=3):
        # Too late to avoid collision at high speed
        take SetBrakeAction(brakeIntensity)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable 4-way signalized intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Vehicle A: Northbound left turn maneuver
leftTurnManeuvers = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
vehicleAManeuver = Uniform(*leftTurnManeuvers)

vehicleATrajectory = [
    vehicleAManeuver.startLane,
    vehicleAManeuver.connectingLane,
    vehicleAManeuver.endLane
]
vehicleAStartLane = vehicleAManeuver.startLane

# Vehicle B: Southbound straight-through maneuver (conflicting with A's left turn)
straightManeuvers = filter(
    lambda m: m.type is ManeuverType.STRAIGHT,
    vehicleAManeuver.conflictingManeuvers
)
vehicleBManeuver = Uniform(*straightManeuvers)

vehicleBTrajectory = [
    vehicleBManeuver.startLane,
    vehicleBManeuver.connectingLane,
    vehicleBManeuver.endLane
]
vehicleBTrajectoryLine = (
    vehicleBManeuver.startLane.centerline +
    vehicleBManeuver.connectingLane.centerline +
    vehicleBManeuver.endLane.centerline
)
vehicleBStartLane = vehicleBManeuver.startLane

# Spawn points along respective lane centerlines
vehicleASpawnPt = new OrientedPoint in vehicleAStartLane.centerline
vehicleBSpawnPt = new OrientedPoint in vehicleBStartLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Vehicle A: Northbound, waiting to turn left
VehicleA = new Car at vehicleASpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior LeftTurnBehavior(
        trajectory=vehicleATrajectory,
        target_speed=globalParameters.OPT_A_SPEED
    )

# Vehicle B: Southbound, accelerating hard to beat the light
VehicleB = new Car at vehicleBSpawnPt,
    with heading vehicleBSpawnPt.heading,
    with regionContainedIn None,
    with behavior AccelerateThroughIntersectionBehavior(
        trajectory=vehicleBTrajectoryLine,
        target_speed=globalParameters.OPT_B_SPEED
    )

# Constraints to ensure proper scenario setup
require monitor TrafficLightController()

# Vehicle A should be positioned near the intersection, ready to turn
require globalParameters.OPT_A_START_DIST[0] <= (distance from vehicleASpawnPt to intersection) <= globalParameters.OPT_A_START_DIST[1]

# Vehicle B should be far enough back to accelerate but close enough to reach intersection
require globalParameters.OPT_B_START_DIST[0] <= (distance from vehicleBSpawnPt to intersection) <= globalParameters.OPT_B_START_DIST[1]

# Ensure vehicles are on opposing approaches (roughly 180 degrees apart)
require 150 deg < abs(vehicleASpawnPt.heading - vehicleBSpawnPt.heading) < 210 deg