"""Scenario Description:

The scenario presents a top-down perspective of a multi-lane urban roadway bordered by grassy areas and dense pine trees, with a central divider separating opposing traffic flows. A blue ego vehicle travels in the right-hand lane behind a red lead vehicle as both approach a four-way intersection marked by crosswalks and traffic signal poles. According to the scenario parameters, the red vehicle is stationary at the intersection stop line, temporarily blocking the lane ahead. The ego vehicle intends to perform a left lane change at the junction but must first hold its position and wait for the red vehicle to accelerate and clear the stop line before safely executing the lateral maneuver. The clear road markings and open surrounding environment provide good visibility, though the ego vehicle's progress is temporarily constrained by the stopped traffic ahead until the intersection clears.

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

EGO_COLOR = (0, 0, 255)       # Blue
LEAD_COLOR = (255, 0, 0)      # Red

EGO_INIT_DIST = [25, 35]
LEAD_STOP_DIST = [2, 5]       # Distance from stop line where lead vehicle stops

param EGO_SPEED = VerifaiRange(6, 9)
param LEAD_ACCEL_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

SAFETY_DIST = 8
CRASH_DIST = 3
TERM_DIST = 80
CLEARANCE_DIST = 15           # Distance lead must travel past stop line before ego proceeds

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(trajectory, stopPoint):
    """Lead vehicle drives to stop line, waits, then accelerates through intersection."""
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_ACCEL_SPEED, trajectory=trajectory)
    interrupt when (distance to stopPoint) < globalParameters.LEAD_STOP_DIST:
        take SetBrakeAction(1.0)
        do WaitUntil(lambda: simulation().currentTime > 3.0)  # Wait at stop line
        take SetThrottleAction(0.8)
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_ACCEL_SPEED, trajectory=trajectory)

behavior EgoLaneChangeBehavior(trajectory, leadVehicle):
    """Ego follows trajectory but brakes if too close to lead; resumes after lead clears."""
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when (distance to leadVehicle) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when (distance to leadVehicle) < globalParameters.CRASH_DIST:
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Select an incoming lane that has a left turn maneuver (representing the right-hand lane
# from which a left lane change / left turn is intended)
egoInitLane = Uniform(*filter(lambda l:
    any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers),
    intersection.incomingLanes))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Lead vehicle is in the same lane as ego, positioned closer to the intersection
leadSpawnPt = new OrientedPoint in egoInitLane.centerline

# Define the stop point at the intersection entrance
stopPoint = new OrientedPoint at intersection.entrancePointFor(egoInitLane)

#################################
# SCENARIO SPECIFICATION        #
#################################

lead = new Car at leadSpawnPt,
    with blueprint LEAD_MODEL,
    with color LEAD_COLOR,
    with behavior LeadVehicleBehavior(egoTrajectory, stopPoint)

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoLaneChangeBehavior(egoTrajectory, lead)

# Ego starts behind lead vehicle at specified distance range
require EGO_INIT_DIST[0] <= (distance from ego to lead) <= EGO_INIT_DIST[1]

# Lead vehicle is near the stop line
require (distance from lead to stopPoint) <= 10

# Ensure ego is behind lead (not ahead)
require (distance from ego to intersection) > (distance from lead to intersection)

terminate when (distance from ego to egoSpawnPt) > TERM_DIST